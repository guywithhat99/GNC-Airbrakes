#include <SD.h>
#include "sd_log_file.hpp"

// -----------------------------------------------------------------------------
// Internal implementation struct (hidden from the header)
// -----------------------------------------------------------------------------
struct sd_log::Impl {
    File logFile;
    bool initialized = false;

    // Store the most recent readings from each sensor
    Vec3 lastMag;
    Vec3 lastGyro;
    Vec3 lastAccel;

    // Flags to track whether each sensor has updated at least once
    bool magReady = false;
    bool gyroReady = false;
    bool accelReady = false;
};

// -----------------------------------------------------------------------------
// Constructor: allocate the Impl object
// -----------------------------------------------------------------------------
sd_log::sd_log() : pimpl(new Impl) {}

// -----------------------------------------------------------------------------
// init(): initialize SD card and open the CSV file
// -----------------------------------------------------------------------------
void sd_log::init() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        return;
    }

    char filename[32];
    int index = 0;

    // Find the first filename of format imu_XXXX.csv that hasn't been claimed
    do {
        snprintf(filename, sizeof(filename), "imu_%04d.csv", index++);
    } while (SD.exists(filename));

    pimpl->logFile = SD.open(filename, FILE_WRITE);
    if (!pimpl->logFile) {
        return; // File could not be opened
    }

    pimpl->initialized = true;

    pimpl->logFile.println(
        "timestamp_us,"
        "mag_x,mag_y,mag_z,"
        "gyro_x,gyro_y,gyro_z,"
        "accel_x,accel_y,accel_z"
    );
}

// -----------------------------------------------------------------------------
// Store magnetometer data (does NOT write to SD yet)
// -----------------------------------------------------------------------------
void sd_log::logMagData(const Vec3& v) {
    if (!pimpl->initialized) return;

    pimpl->lastMag = v;
    pimpl->magReady = true;
}

// -----------------------------------------------------------------------------
// Store gyroscope data
// -----------------------------------------------------------------------------
void sd_log::logGyroData(const Vec3& v) {
    if (!pimpl->initialized) return;

    pimpl->lastGyro = v;
    pimpl->gyroReady = true;
}

// -----------------------------------------------------------------------------
// Store accelerometer data
// -----------------------------------------------------------------------------
void sd_log::logAccelData(const Vec3& v) {
    if (!pimpl->initialized) return;

    pimpl->lastAccel = v;
    pimpl->accelReady = true;
}

// -----------------------------------------------------------------------------
// Write a combined row using the *latest available* values.
// Missing readings simply reuse the last known value.
// -----------------------------------------------------------------------------
void sd_log::writeCombinedRow() {
    if (!pimpl->initialized) return;

    // Only block logging until each sensor has produced *at least one* reading.
    if (!(pimpl->magReady && pimpl->gyroReady && pimpl->accelReady)) {
        return;
    }

    uint32_t t = micros(); // timestamp in microseconds

    // Write one complete IMU snapshot
    pimpl->logFile.printf(
        "%lu,"
        "%f,%f,%f,"      // mag
        "%f,%f,%f,"      // gyro
        "%f,%f,%f\n",    // accel
        t,
        pimpl->lastMag.x,   pimpl->lastMag.y,   pimpl->lastMag.z,
        pimpl->lastGyro.x,  pimpl->lastGyro.y,  pimpl->lastGyro.z,
        pimpl->lastAccel.x, pimpl->lastAccel.y, pimpl->lastAccel.z
    );

    // IMPORTANT:
    // We do NOT reset the ready flags.
    // This allows reuse of last known values if a sensor doesn't update next cycle.
}
