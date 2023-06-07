#include "transmitter.h"

// Define program states
enum RocketState
{
  IDLE,
  ARMED,
  CLI
};

// initial state to IDLE
RocketState state = IDLE;

// Declare sensor objects
Adafruit_BMP3XX bmp;

// Variables to store IMU data
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// see the the LSM6DS_LIS3MDL file in this project to change board to LSM6DS33, LSM6DS3U, LSM6DSOX, etc
#include "LSM6DS_LIS3MDL.h"
sensors_event_t mag_event, gyro_event, accel_event;

// Loop count and time conditions
unsigned long start_time; // time for loop

// GPS Initilization
SFE_UBLOX_GNSS myGPS;
int counter = 0; // Disable the GPS debug messages when counter reaches 20

// Initialize I2C bus
TwoWire I2CBUS = TwoWire(3);

long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends
bool led_flag = 0;

// listens for the SA command to arm the board
bool check_if_armed()
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

      return true;
    }
  }

  return false;
}

// Sends a String over LoRa
void sendPacket(String message)
{
  LoRa.beginPacket();
  LoRa.setTxPower(20,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print(message);
  LoRa.endPacket();
}


void lora_task(void *params) {

  while(1) {

    switch (state) {
      case (IDLE): {

        Heltec.display->clear();
        char buff[40];

        long latitude = myGPS.getLatitude();
        sprintf(buff, "Lat: %i", latitude);
        Heltec.display->drawString(0, 0, buff);

        long longitude = myGPS.getLongitude();
        sprintf(buff, "Long: %i", longitude);
        Heltec.display->drawString(0, 10, buff);

        // Read pressure sensor (float)
        if (!bmp.performReading()) {
          return;
        }
        float bmp_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        // float bmp_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        sprintf(buff, "Altitude: %.2f m", bmp_altitude);
        Heltec.display->drawString(0,20,buff);

        sprintf(
          buff,
          "%i/%i/%i %i:%i:%i",
          myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(), myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond()
        );
        Heltec.display->drawString(0, 30, buff);

        sprintf(buff, "SkyNet IDLE: %i", counter);
        Heltec.display->drawString(0, 40, buff);

        Heltec.display->display(); // display all the above display calls

        // Check if rocket should be armed
        if (check_if_armed()) {
          // Serial.println("STATUS CHANGE - ARMED");
          state = ARMED;
          Heltec.display->clear();
        }
        else {
          if (millis() - lastSendTime > interval) {
            lastSendTime = millis();
            interval = random(2000) + 1000;    // 1-3 seconds    // Reset last send time
            sendPacket("IDLE");
          }
        }
        break;
      }

      case(ARMED): {

        sendPacket("ARMD");
        char buff[40];

        // Read pressure sensor (float)
        if (!bmp.performReading()){
          return;
        }

        // Read time of flight sensor
        uint16_t tof_data = 0; // unimplemented

        // read IMU
        magnetometer->getEvent(&mag_event);
        gyroscope->getEvent(&gyro_event);
        accelerometer->getEvent(&accel_event);

        // Store BMP data
        float temptemperature = bmp.temperature;
        float temppressure = bmp.pressure;
        float tempaltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

        // Send all sensor data over LoRa
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

        break;
      }

      case(CLI): {
        led_flag = 1^led_flag;
        digitalWrite(LED_BUILTIN,led_flag);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        break;
      }

    }
    counter ++;
  }

}


void setup(void) {

  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(0,INPUT);
  // if (not digitalRead(0)) {
  //   state = CLI;
  // }

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
  delay(1000); // wait for board to be initialized
  LoRa.setSignalBandwidth(500E3);

  Serial.begin(115200); // init serial for testing
  while (!Serial) {
    delay(10);
  }

  Heltec.VextON(); // Enable 3.3V Vext
  I2CBUS.begin(13, 22, 100000); // start I2C bus

  // Initialize GPS
  if (myGPS.begin(I2CBUS, 0x42, 1100, false) == false)
  {
    // Serial.println(F("Failed to find Ublox GPS"));
    Heltec.display->drawString(0, 20, "Failed to find Ublox GPS");
    Heltec.display->display();
    while (1) {;}
  }
  else
  {
    // Serial.println(F("Booted GPS"));
    Heltec.display->drawString(0, 20, "Booted GPS");
    Heltec.display->display();
  }

  // Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setI2COutput(COM_TYPE_UBX);
  myGPS.enableDebugging();

  // Initialize BMP388 pressure sensor
  if (!bmp.begin_I2C(0x77, &I2CBUS))
  {
    // Serial.println(F("Failed to boot BMP388"));
    Heltec.display->drawString(0, 30, "Failed to boot BMP388");
    Heltec.display->display();
    while (1) {;}
  }
  else
  {
    // Serial.println(F("Booted BPM388"));
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
    while (1) {
      delay(10);
    }
  }
  else
  {
    Serial.println(F("Booted IMU sensors"));
    Heltec.display->drawString(0, 40, "Booted IMU sensors");
    Heltec.display->display();
  }

  // Print out sensor information
  // accelerometer->printSensorDetails();
  // gyroscope->printSensorDetails();
  // magnetometer->printSensorDetails();

  setup_sensors();

  Wire.setClock(400000); // 400KHz

  // create task for LoRa communication
  xTaskCreate(
      lora_task,    // function
      "LoRa task",  // task name
      2048,         // stack size
      NULL,         // task parameters
      1,            // task priority
      NULL          // task handle
  );

  // Create another task for fast data logging to flash memory

}

void loop() {}
