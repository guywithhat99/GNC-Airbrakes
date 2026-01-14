// Basic demo for accelerometer readings from Adafruit ICM20948

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_ICM20948 icm;
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
// For SPI mode, we need a CS pin
#define ICM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define ICM_SCK 13
#define ICM_MISO 12
#define ICM_MOSI 11

int count = 1;
float wx_b = 0, wy_b = 0, wz_b = 0;
float soak_start = 1; // # of seconds to start calibration sweep
float soak_end = 120; // # of seconds to calibrate gyro-drift
const int num_tests = 10;
float soak_times[num_tests];
bool flag = false;
float soak_slope = 0;
float sum = 0;
int test_idx = 0;
float test_t0 = 0;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20948 Found!");

  // Change Sensor Resolution
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();

  soak_slope = (soak_end - soak_start) / (num_tests - 1);
  for (int i = 0; i < num_tests; i++)
  {
    soak_times[i] = soak_start + (soak_slope * i);
  }
  for (int i = 0; i < num_tests; i++) 
  {
    sum += soak_times[i];
  }
  Serial.println("Soaking Sweep Program");
  Serial.print("Initial Soak Time: ");
  Serial.print(soak_start);
  Serial.println(" seconds");
  Serial.print("Final Soak Time: ");
  Serial.print(soak_end);
  Serial.println(" seconds");
  Serial.print("Number of Tests: ");
  Serial.println(num_tests);
  Serial.print("Approximate Time to Complete Sweep: ");
  Serial.println(sum);
  test_t0 = micros() * 1e-6f;
}

void loop() {
  // Check if we are finished
  if (test_idx >= num_tests)
  {
    Serial.println("Sweep Complete");
    while(true)
    {
      delay(1000);
    }
  }
  // Get Sensor Data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag;
  sensors_event_t temp;
  icm.getEvent(&accel, &gyro, &temp, &mag);


  // Assign sensor data to variables
  float wx = gyro.gyro.x;
  float wy = gyro.gyro.y;
  float wz = gyro.gyro.z;
  float t_current = micros() * 1e-6f;

  // Compute Elapsed Test Time
  float elapsed = t_current - test_t0;

  // Are we still running the test?
      if (elapsed < soak_times[test_idx])
      {
        // Edge Case 
        if (count == 1)
        {
          wx_b = wx;
          wy_b = wy;
          wz_b = wz;
          count++;
        }
        else
        {
          wx_b = wx_b + (1.0f / count) * (wx - wx_b);
          wy_b = wy_b + (1.0f / count) * (wy - wy_b);
          wz_b = wz_b + (1.0f / count) * (wz - wz_b);
          count++;
        }
        // Add Delay to Match Loop Clock with Gyro Sensor ODR
        delay(10);
      }
      else
      {
        // Print Results
        Serial.print("Current Soak Test Time: ");
        Serial.println(soak_times[test_idx]);
        Serial.println("Values Found: ");
        Serial.print("X Angular Velocity Offest: ");
        Serial.print(wx_b * 1000.0f);
        Serial.print(" milli-rad / second \n");
        Serial.print("Y Angular Velocity Offest: ");
        Serial.print(wy_b * 1000.0f);
        Serial.print(" milli-rad / second \n");
        Serial.print("\n");
        Serial.print("Z Angular Velocity Offset: ");
        Serial.print(wz_b * 1000.0f);
        Serial.print(" milli-rad / second \n");
        Serial.println();
        delay(1);

        // Advance test
        test_idx++;
        count = 1;
        wx_b = wy_b = wz_b = 0;
        delay(500);
        test_t0 = micros() * 1e-6f;
      }
}
