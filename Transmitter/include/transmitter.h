// Project header file which includes important headers and defines some constants
#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Adafruit_BMP3XX.h>
#include <cstring>
#include "heltec.h"
#include "ToF.h"
#include "utils.hpp"
#include "filesys.hpp"

#define BAND 915E6  // You can set band here directly,e.g. 868E6,915E6
int SEALEVELPRESSURE_HPA = 1013;

#define DT  0.02          // Loop time
#define AA  0.97         // complementary filter constant
#define G_GAIN 0.070    // [deg/s/LSB]

#ifndef M_PI
  #define M_PI 3.141592653589793
#endif
