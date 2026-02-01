/**
 * GNC-Airbrakes Firmware
 * Teensy 4.1 Entry Point
 */

#include <Arduino.h>
#include "imu.hpp"
#include "sd_log_file.hpp"

IMU imu;
sd_log sdLog;

void setup() {
    Serial.begin(115200);
    delay(500);

    imu.init(IMU::defaultConfig());

    Serial.println("GNC-Airbrakes firmware initialized");
}

void loop() {
    imu.update();
    Vec3 gyro;
    Vec3 accel;
    Vec3 mag;


    //if there is new data from the gyro on the imu sensor
    if (imu.gyroReady()) {
        gyro = imu.readGyro();
        sdLog.logGyroData(gyro); 
    }

    //if there is new data from the acceleromoter on the imu sensor
    if (imu.accelReady()) {
        accel = imu.readAccel();
        
    }

    //if there is new data from the magnetomter on the imu sensor
    if (imu.magReady()) {
        mag = imu.readMag();

    }

    
}
