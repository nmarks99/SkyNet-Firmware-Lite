// Project header file which includes important headers and defines some constants
#include <Adafruit_BMP3XX.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include <cstring>

#include "ToF.h"
#include "filesys.hpp"
#include "heltec.h"
#include "utils.hpp"

#define BAND 915E6  // You can set band here directly,e.g. 868E6,915E6
int SEALEVELPRESSURE_HPA = 1013;
constexpr float FEET_PER_METER = 3.280839895;
constexpr uint16_t SENSOR_POLL_PERIOD_MS = 20;  // time in ms between each sensor read
constexpr uint16_t GPS_POLL_PERIOD_MS = 1000;   // probably can be even slower

// #define DT 0.02       // Loop time
// #define AA 0.97       // complementary filter constant
// #define G_GAIN 0.070  // [deg/s/LSB]

// #ifndef M_PI
// #define M_PI 3.141592653589793
// #endif
