
#include "sd_log_file.hpp"
#include "imu.hpp"
#include <time.h> // TODO: change to teensy4 Time header




void sd_log::logGyroData(const Vec3& data) {
  readtofile(data)
  return;
}
