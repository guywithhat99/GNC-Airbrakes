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
  char sensor_string_buff[128];

  // Must call this often in main loop -- updates the sensor values
  icm20948.task();

  if (icm20948.gyroDataIsReady())
  {
    icm20948.readGyroData(&gyro_x, &gyro_y, &gyro_z);
    sprintf(sensor_string_buff, "Gyro (deg/s): [%f,%f,%f]", gyro_x, gyro_y, gyro_z);
    Serial.println(sensor_string_buff);
  }

  if (icm20948.accelDataIsReady())
  {
    icm20948.readAccelData(&accel_x, &accel_y, &accel_z);
    sprintf(sensor_string_buff, "Accel (g): [%f,%f,%f]", accel_x, accel_y, accel_z);
    Serial.println(sensor_string_buff);
  }

  if (icm20948.magDataIsReady())
  {
    icm20948.readMagData(&mag_x, &mag_y, &mag_z);
    sprintf(sensor_string_buff, "Mag (uT): [%f,%f,%f]", mag_x, mag_y, mag_z);
    Serial.println(sensor_string_buff);
  }

  if (icm20948.quatDataIsReady())
  {
    icm20948.readQuatData(&quat_w, &quat_x, &quat_y, &quat_z);
    sprintf(sensor_string_buff, "Quat (deg): [%f,%f,%f,%f]", quat_w, quat_x, quat_y, quat_z);
    Serial.println(sensor_string_buff);
  }
}
