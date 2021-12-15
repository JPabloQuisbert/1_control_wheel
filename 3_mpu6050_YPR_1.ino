#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu(0x68);


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/////////////////////DEFINE PID constants/////////////////////
//////////////////////////////////////////////////////////////
int kp = 20, ki = 0.01, kd = 0.02;
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

int input, output, setpoint;
int iTerm = 0, lastInput = 0, dInput = 0, error = 0;
int outMin = -255, outMax = 255;
int sampleTime = 10;                                //This values is is milliseconds
volatile long encoderPos = 0;

//////////////////////////////////////////////////////////////
////////////////////Define the pins we use////////////////////
//////////////////////////////////////////////////////////////
#define Encoder_A      2                                    // Quadrature encoder A pin
#define Encoder_B      8                                    // Quadrature encoder B pin
#define Motor_CW       11                                   // PWM outputs to L298N H bridge motor driver module
#define Motor_CCW      3
#define led            13
#define END_stop       4


void Compute(float sensor)
{
  
  //setpoint = map(analogRead(0),0,1024,1024,0) * 110;            // setpoint position is made with a potentiometer but could be given by serial monitor or other...
  setpoint = 10;
  input = sensor;                                         // we get the data from the encoder interrumption
  error = setpoint - input;
  iTerm += ki * error * sampleTime;
  if (iTerm > outMax) iTerm = outMax;                           // prevent that the I term from PID gets too big
  else if (iTerm < outMin) iTerm = outMin;
  dInput = (input - lastInput) / sampleTime;
  output = kp * error + iTerm + kd * dInput;                    // The PID output is the sum of P I and D values
  if (output > outMax) output = outMax;                         // limit output to 0 and 255 for the analog write
  else if (output < outMin) output = outMin;          
  lastInput = input;                                            //Remember to save the last input value for the next loop
  pwmOut(output);                                               //Change the analog write for the motor control
  //Serial.print("entrada: ");
//  Serial.println(input);
//  Serial.print("salida: ");
//  Serial.println(output);
}
void pwmOut(int out) {                                          // to H-Bridge board
  if (out > 0) {
    analogWrite(Motor_CW, out);                                 // Rotate the motor CW
    analogWrite(Motor_CCW, 0);
  }
  else {
    analogWrite(Motor_CW, 0);
    analogWrite(Motor_CCW, abs(out));                           // Rotate the motor CCW
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);// 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(38400);
//  Serial.println("Initializing I2C devices...");
  mpu.initialize();
//  Serial.println(F("Testing device connections..."));
//  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//  //-3226  -827  1394
//  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//  Serial.print("\n");
//  accelgyro.setXAccelOffset(0);
//  accelgyro.setYAccelOffset(0);
//  accelgyro.setZAccelOffset(0);
////  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
////  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
////  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
////  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
////  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
////  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//  Serial.print("\n");
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(58);
  mpu.setYGyroOffset(-3);
  mpu.setZGyroOffset(35);
  mpu.setXAccelOffset(-4942);
  mpu.setYAccelOffset(-1039);
  mpu.setZAccelOffset(2854);
  if(devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
//    mpu.PrintActiveOffsets();
//    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    
//    mpuIntStatus = mpu.getIntStatus();
//    dmpReady = true; //not really necesasry
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  pinMode(Encoder_A, INPUT);                                // quadrature encoder input A
  pinMode(Encoder_B, INPUT);                                // quadrature encoder input B
  pinMode(END_stop, INPUT);                                 // Input from the end stop switch
  pinMode(led, OUTPUT);
  
}

void loop() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("y\t");
    Serial.println(ypr[0] * 180 / M_PI);
//    Serial.print("\tp: \t");
//    Serial.print(ypr[1] * 180 / M_PI);
//    Serial.print("\tr: \t");
//    Serial.println(ypr[2] * 180 / M_PI);
    Compute(ypr[0] * 180 / M_PI);
  }
  delay(10);

}
