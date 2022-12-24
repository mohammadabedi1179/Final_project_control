#include "I2Cdev.h"
#include "math.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"

#endif
MPU6050 mpu;
  
#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_YAWPITCHROLL

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer [64];
double yaw ,pitch ,roll ,sinp;
int sign; 
Quaternion q;
double pii =3.1415926535897932384626433832795;

void setup() {
  
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
Wire.begin();
//Wire.setClock (400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
Fastwire ::setup (400, true); 
#endif

Serial.begin(115200);
//while (!Serial);
//Serial.println(F("Initializing_I2C_devices ..."));

mpu.initialize ();
//Serial.println(F("Testing_device_connections ..."));
//Serial.println(mpu.testConnection () ?
//F("MPU6050_connection_successful") : 
//F("MPU6050_connection_failed"));
//Serial.println(F("Initializing DMP..."));
devStatus = mpu.dmpInitialize ();
mpu.setXGyroOffset (220);
mpu.setYGyroOffset (76); 
mpu.setZGyroOffset (-85);
mpu.setZAccelOffset (1788);
if (devStatus == 0) {
mpu.CalibrateAccel (6);
//Serial.println("kir");
mpu.CalibrateGyro (6);
//Serial.println("kir");
mpu.PrintActiveOffsets ();
//Serial.println(F("Enabling DMP..."));
mpu.setDMPEnabled(true);
dmpReady = true;
}

else { 
Serial.print(F("DMP Initialization failed (code"));
Serial.print(devStatus); 
Serial.println(F(")")); 
} 
} 

void loop() {
   
if (! dmpReady) return;
if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
mpu.dmpGetQuaternion (&q, fifoBuffer);
 
roll = atan2 (2*(q.w*q.y+q.x*(-q.z)), 1-2*(q.y*q.y+q.x*q.x));
 
sinp = 2*(q.w*q.x-(-q.z*q.y));

if abs(sinp >=1){
if (sinp < 0) sign= -1;
else if (sinp ==0) sign = 0;
else sign= 1;
pitch = pii/2* sign;
}
else pitch = asin(sinp);
yaw = atan2 (2*(q.w*(-q.z)+q.y*q.x), 1-2*(q.x*q.x+q.z*q.z));
yaw = (int) (yaw *180/ pii) + 0;
pitch = (int) (pitch *180/ pii) + 0;
roll = (int) (roll *180/ pii) + 0;
Serial.print("roll");Serial.print(" ");
Serial.print(roll);Serial.print("\t");
Serial.print("yaw");Serial.print(" ");Serial.print(yaw);Serial.print("\t");
Serial.print("pitch"); Serial.print(" ");Serial.println(pitch);
}
}
