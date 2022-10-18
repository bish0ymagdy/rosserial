#include <MPU6050.h>
#include <ros.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "I2Cdev.h"
#include <std_msgs/Float32.h>
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
uint8_t devStatus;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;

ros::NodeHandle nh;
std_msgs::Float32 yaw_msg;
ros::Publisher yaw("yaw", &yaw_msg);

void setup() {
  nh.initNode();
  nh.advertise(yaw);

  Serial.begin(115200);
  while (!Serial);
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

}

void loop() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Serial.print("yaw\t");
  Serial.print(ypr[0] * 180 / M_PI);
  yaw_msg.data = ypr[0] * 180 / M_PI;
  yaw.publish( &yaw_msg );
  nh.spinOnce();
  delay(1000);

}
