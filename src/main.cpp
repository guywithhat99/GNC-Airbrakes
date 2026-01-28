/**
 * GNC-Airbrakes Firmware
 * Teensy 4.1 Entry Point
 */

#include <Arduino.h>
#include "Teensy-ICM-20948.h"
TeensyICM20948 icm20948;

TeensyICM20948Settings icmSettings =
{
  .cs_pin = 24,                  // SPI chip select pin
  .spi_speed = 1000000,          // SPI clock speed in Hz, max speed is 7MHz
  .mode = 1,                     // 0 = low power mode, 1 = high performance mode
  .enable_gyroscope = true,      // Enables gyroscope output
  .enable_accelerometer = true,  // Enables accelerometer output
  .enable_magnetometer = true,   // Enables magnetometer output
  .enable_quaternion = true,     // Enables quaternion output
  .gyroscope_frequency = 1,      // Max frequency = 225, min frequency = 1
  .accelerometer_frequency = 1,  // Max frequency = 225, min frequency = 1
  .magnetometer_frequency = 1,   // Max frequency = 70, min frequency = 1
  .quaternion_frequency = 50     // Max frequency = 225, min frequency = 50
};

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(500);

   icm20948.init(icmSettings);

    Serial.println("GNC-Airbrakes firmware initialized");
}

void loop() {
  float gyro_x, gyro_y, gyro_z;
  float accel_x, accel_y, accel_z;
  float mag_x, mag_y, mag_z;
  float quat_w, quat_x, quat_y, quat_z;

  // Must call this often in main loop -- updates the sensor values
  icm20948.task();

  if (icm20948.gyroDataIsReady())
  {
    icm20948.readGyroData(&gyro_x, &gyro_y, &gyro_z);
    Serial.print("Gyro (deg/s): [");
    Serial.print(gyro_x); Serial.print(",");
    Serial.print(gyro_y); Serial.print(",");
    Serial.print(gyro_z); Serial.println("]");
  }

  if (icm20948.accelDataIsReady())
  {
    icm20948.readAccelData(&accel_x, &accel_y, &accel_z);
    Serial.print("Accel (g): [");
    Serial.print(accel_x); Serial.print(",");
    Serial.print(accel_y); Serial.print(",");
    Serial.print(accel_z); Serial.println("]");
  }

  if (icm20948.magDataIsReady())
  {
    icm20948.readMagData(&mag_x, &mag_y, &mag_z);
    Serial.print("Mag (uT): [");
    Serial.print(mag_x); Serial.print(",");
    Serial.print(mag_y); Serial.print(",");
    Serial.print(mag_z); Serial.println("]");
  }

  if (icm20948.quatDataIsReady())
  {
    icm20948.readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);
    Serial.print("Quat: [");
    Serial.print(quat_w); Serial.print(",");
    Serial.print(quat_x); Serial.print(",");
    Serial.print(quat_y); Serial.print(",");
    Serial.print(quat_z); Serial.println("]");
  }
}
