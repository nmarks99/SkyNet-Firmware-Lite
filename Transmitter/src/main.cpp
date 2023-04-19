#include "transmitter.h"

// Define program states
enum RocketState
{
  IDLE,
  ARMED,
  LAUNCHED,
  RECOVERY
};

RocketState currentState = LAUNCHED;

const float LAUNCH_THRESHOLD = 12.5; // Lower threshold acceleration (in m/s^2) to determine launch
const float RECOVERY_THRESHOLD = 10.2; // Upper threshold acceleration (in m/s^2) to determine the rocket is landed

// Declare sensor objects
Adafruit_VL53L0X ToF = Adafruit_VL53L0X(); // time of flight
Adafruit_BMP3XX bmp;                       // pressure

// variables to store IMU data
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer; // IMU

// see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
#include "LSM6DS_LIS3MDL.h"
sensors_event_t mag_event, gyro_event, accel_event;

// Loop count and time conditions
unsigned long startTime; // time for loop
int loopcount = 0;

// GPS Initilization
SFE_UBLOX_GNSS myGPS;
int counter = 0; // Disable the GPS debug messages when counter reaches 20

TwoWire I2CBUS = TwoWire(3);

long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

/*--------------------------------------------------*/
void setup(void)
{

  // Start Heltec LoRa ESP32 Board
  Heltec.begin(
      true /*DisplayEnable Enable*/,
      true /*LoRa enable*/,
      true /*Serial Enable*/,
      true /*PABOOST Enable*/,
      BAND /*long BAND*/);

  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "SkyNet Transmitter Started");
  Heltec.display->display();

  delay(1000);          // wait for board to be initialized
                        //
  LoRa.setSignalBandwidth(500E3);
  Serial.begin(115200); // init serial for testings
  while (!Serial)
    delay(10);

  Heltec.VextON(); // Enable 3.3V Vext
  I2CBUS.begin(13, 22, 100000);

  // Initialize time of flight sensor
  if (!ToF.begin(0x29, false, &I2CBUS))
  {
    Serial.println(F("Failed to boot VL53L0X"));
    Heltec.display->drawString(0, 10, "Failed to boot VL53L0X");
    Heltec.display->display();
    while (1)
    {
      ;
    }
  }
  else
  {
    Serial.println(F("Booted VL53L0X"));
    Heltec.display->drawString(0, 10, "Booted VL53L0X");
    Heltec.display->display();
  }

  // Initialize GPS
  if (myGPS.begin(I2CBUS, 0x42, 1100, false) == false) // Connect to the Ublox module using Wire port
  {
    Serial.println(F("Failed to find Ublox GPS"));
    Heltec.display->drawString(0, 20, "Failed to find Ublox GPS");
    Heltec.display->display();
    while (1)
      ;
  }
  else
  {
    Serial.println(F("Booted GPS"));
    Heltec.display->drawString(0, 20, "Booted GPS");
    Heltec.display->display();
  }

  myGPS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.enableDebugging();

  // Initialize BMP388 pressure sensor
  if (!bmp.begin_I2C(0x77, &I2CBUS))
  {
    Serial.println(F("Failed to boot BMP388"));
    Heltec.display->drawString(0, 30, "Failed to boot BMP388");
    Heltec.display->display();
    while (1)
      ;
  }
  else
  {
    Serial.println(F("Booted BPM388"));
    Heltec.display->drawString(0, 30, "Booted BPM388");
    Heltec.display->display();
  }

  // Set up oversampling and filter initialization on the BMP388
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Uncomment Serial.print statements for debugging

  // Initialize IMU sensors
  if (!init_sensors(&I2CBUS))
  {
    Serial.println(F("Failed to find IMU sensors"));
    Heltec.display->drawString(0, 40, "Failed to find IMU sensors");
    Heltec.display->display();
    while (1)
      delay(10);
  }
  else
  {
    Serial.println(F("Booted IMU sensors"));
    Heltec.display->drawString(0, 40, "Booted IMU sensors");
    Heltec.display->display();
  }

  // Print out sensor information
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();

  setup_sensors();

  Wire.setClock(400000); // 400KHz
}

bool checkIfArmed()
{
  // Check for incoming packets
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    // Read packet
    String packet = "";
    while (LoRa.available())
    {
      packet += (char)LoRa.read();
    }
    Serial.print("Incoming LoRa Packet: ");
    Serial.println(packet);
    // Check if packet starts with SA (Switchstate Armed)
    if (packet.startsWith("SA"))
    {
      // Switchstate Armed received
      // Get the four characters after SA for SEALEVELPRESSURE_HPA
      String pressure = packet.substring(2, 6);
      // Convert to integer
      SEALEVELPRESSURE_HPA = pressure.toInt();
      // Print out the new SEALEVELPRESSURE_HPA
      Serial.print("New SEALEVELPRESSURE_HPA: ");
      Serial.println(SEALEVELPRESSURE_HPA);
      // Return true to indicate that the rocket is armed
      return true;
    }
  }

  return false;
}

bool checkIfLaunched()
{
  // Read accelerometer data
  if (!accelerometer->getEvent(&accel_event))
  {
    // Problem reading accelerometer data
    Serial.println(F("Error reading accelerometer data"));
  }

  // Use "Y" axis marked by the IMU board for launch detection
  float acceleration = accel_event.acceleration.y;
  Serial.print("Acceleration: ");
  Serial.println(acceleration);
  // Check if launch threshold has been exceeded
  return acceleration > LAUNCH_THRESHOLD;
}

bool checkIfLanded()
{
  // read accelerometer data
  if (!accelerometer->getEvent(&accel_event))
  {
    // Problem reading accelerometer data
    Serial.println(F("Error reading accelerometer data"));
  }

  // Check if acceleration on xyz is smaller than target ms^2
  return (abs(accel_event.acceleration.x) + 
          abs(accel_event.acceleration.y) + 
          abs(accel_event.acceleration.z)) < RECOVERY_THRESHOLD;
  // Placeholder function that always returns false
  return false;
}

void sendPacket(String message)
{
  LoRa.beginPacket();
  LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(message);
  LoRa.endPacket();
}

void loop()
{
  startTime = millis();

  // clear the display
  Heltec.display->clear();

  // Read pressure sensor (float)
  if (!bmp.performReading())
  {
    return;
  }

  // Read time of flight sensor (unsigned short)
  VL53L0X_RangingMeasurementData_t val = read_ToF(ToF);
  uint16_t tof_data = 0;
  if (val.RangeStatus != 4)
  {
    tof_data = val.RangeMilliMeter;
  }
  else
  {
    // if data is bad (probably nothing in range) set equal to zero
    tof_data = 0;
  }

  // Read IMU data (all floats)
  magnetometer->getEvent(&mag_event);
  gyroscope->getEvent(&gyro_event);
  accelerometer->getEvent(&accel_event);

  // Pack up the data into an array
  char buffer[150];
  char data_format[] = "%hu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f";

  sprintf(buffer, data_format,
          tof_data,
          bmp.temperature,
          bmp.pressure,
          bmp.readAltitude(SEALEVELPRESSURE_HPA),
          accel_event.acceleration.x,
          accel_event.acceleration.y,
          accel_event.acceleration.z,
          gyro_event.gyro.x,
          gyro_event.gyro.y,
          gyro_event.gyro.z,
          mag_event.magnetic.x,
          mag_event.magnetic.y,
          mag_event.magnetic.z);

  float temptemperature = bmp.temperature;
  float temppressure = bmp.pressure;
  float tempaltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  LoRa.beginPacket();
  LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST);
  LoRa.write((uint8_t)tof_data);
  LoRa.write((uint8_t *)&temptemperature, 4);
  LoRa.write((uint8_t *)&temppressure, 4);
  LoRa.write((uint8_t *)&tempaltitude, 4);
  LoRa.write((uint8_t *)&accel_event.acceleration.x, 4);
  LoRa.write((uint8_t *)&accel_event.acceleration.y, 4);
  LoRa.write((uint8_t *)&accel_event.acceleration.z, 4);
  LoRa.write((uint8_t *)&gyro_event.gyro.x, 4);
  LoRa.write((uint8_t *)&gyro_event.gyro.y, 4);
  LoRa.write((uint8_t *)&gyro_event.gyro.z, 4);
  LoRa.write((uint8_t *)&mag_event.magnetic.x, 4);
  LoRa.write((uint8_t *)&mag_event.magnetic.y, 4);
  LoRa.write((uint8_t *)&mag_event.magnetic.z, 4);
  LoRa.endPacket();

  Heltec.display->display();
  counter++; // Increment counter
  // Each loop should be at least 20ms.
  while (millis() - startTime < (DT * 1000))
  {
    // wait
  }

} // END LOOP
