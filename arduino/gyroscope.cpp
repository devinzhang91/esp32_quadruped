#include "pca_action.h"
#include "pid.h"

#include "I2Cdev.h"
#include "kalman.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

MPU6050 mpu = MPU6050(MPU6050_ADDRESS_AD0_HIGH);

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

int initGyro(bool use_dmp){
    mpu.initialize();
    if(mpu.testConnection()){
        Serial.println("MPU6050 connection successful");
    } else {
        Serial.println("MPU6050 connection failed");
        return -1;
    }

    if(use_dmp){
        // load and configure the DMP
        Serial.println("Initializing DMP...");
        if(! mpu.dmpInitialize() ){
            // supply your own gyro offsets here, scaled for min sensitivity
            mpu.setXGyroOffset(23);
            mpu.setYGyroOffset(-2);
            mpu.setZGyroOffset(71);
            mpu.setZAccelOffset(1158); // 1688 factory default for my test chip
            // Calibration Time: generate offsets and calibrate our MPU6050
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            Serial.println();
            mpu.PrintActiveOffsets();
            Serial.println("Enabling DMP...");
            mpu.setDMPEnabled(true);
            mpuIntStatus = mpu.getIntStatus();
            dmpReady = true;
            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
            Serial.println("DMP ready! Waiting for read fifo ...");
        } else {
            Serial.println("DMP Initialization failed");
            return -2;
        }
    } else {
        mpu.setXGyroOffset(23);
        mpu.setYGyroOffset(-2);
        mpu.setZGyroOffset(71);
        mpu.setZAccelOffset(1158); // 1688 factory default for my test chip
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
    }
    return 0;
}

int getYPRDMP(float *ypr){
    if(dmpReady == false ) {
        Serial.println("DMP NOT ready.");
        return -1;
    }
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Serial.print("ypr\t");
        // Serial.print(ypr[0] * 180/M_PI);
        // Serial.print("\t");
        // Serial.print(ypr[1] * 180/M_PI);
        // Serial.print("\t");
        // Serial.println(ypr[2] * 180/M_PI);
    } else {
        // Serial.println("DMP FIFO empty.");
        return -2;
    }
    return 0;
}

Kalman pitch_kalman(SAFE_T*2);
Kalman roll_kalman(SAFE_T*2);

int getYPRKalman(float *ypr){
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float angleAx=atan2(ax,az)*180/PI;//计算与x轴夹角  
    float gyroGy=-gy/131.00;//计算角速度 
    float pitch = pitch_kalman.run(angleAx, gyroGy);

    float angleAy=atan2(ay,az)*180/PI;//计算与y轴夹角  
    float gyroGz=gz/131.00;//计算角速度 
    float roll = roll_kalman.run(angleAy, gyroGz);

    ypr[1] = pitch;
    ypr[2] = roll;

    return 0;
}