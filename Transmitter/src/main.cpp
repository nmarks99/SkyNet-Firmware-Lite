#include "transmitter.h"

// Define program states
enum RocketState {
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
// unsigned long start_time;  // time for loop

// GPS Initilization
SFE_UBLOX_GNSS myGPS;
int counter = 0;  // Disable the GPS debug messages when counter reaches 20

// Initialize I2C bus
TwoWire I2CBUS = TwoWire(3);

long lastSendTime = 0;  // last send time
int interval = 2000;    // interval between sends
bool led_flag = 0;

// Global variables to store sensor data
volatile float pressure = 0.0;
volatile float temperature = 0.0;
volatile float altitude = 0.0;
volatile float gyro_x = 0.0;
volatile float gyro_y = 0.0;
volatile float gyro_z = 0.0;
volatile float acc_x = 0.0;
volatile float acc_y = 0.0;
volatile float acc_z = 0.0;
volatile float mag_x = 0.0;
volatile float mag_y = 0.0;
volatile float mag_z = 0.0;
volatile int32_t latitude = 0;
volatile int32_t longitude = 0;

float altitude_zero = 0.0;  // used to zero the altitude at ground level

static SemaphoreHandle_t mutex;
static SemaphoreHandle_t mutex_gps;

// listens for the SA command to arm the board
bool check_if_armed() {
    // Check for incoming packets
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        // Read packet
        String packet = "";
        while (LoRa.available()) {
            packet += (char)LoRa.read();
        }
        // Serial.print("Incoming LoRa Packet: ");
        // Serial.println(packet);

        // Check if packet starts with SA (Switchstate Armed)
        if (packet.startsWith("SA")) {
            // Switchstate Armed received
            // Get the four characters after SA for SEALEVELPRESSURE_HPA
            String pressure = packet.substring(2, 6);

            // Convert to integer
            SEALEVELPRESSURE_HPA = pressure.toInt();

            // Print out the new SEALEVELPRESSURE_HPA
            // Serial.print("New SEALEVELPRESSURE_HPA: ");
            // Serial.println(SEALEVELPRESSURE_HPA);

            return true;
        }
    }

    return false;
}

// Sends a String over LoRa
void sendPacket(String message) {
    LoRa.beginPacket();
    LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST);
    LoRa.print(message);
    LoRa.endPacket();
}

void main_loop(void *params) {
    float tmp_pressure = 0.0;
    float tmp_temperature = 0.0;
    float tmp_altitude = 0.0;
    float tmp_gyro_x = 0.0;
    float tmp_gyro_y = 0.0;
    float tmp_gyro_z = 0.0;
    float tmp_acc_x = 0.0;
    float tmp_acc_y = 0.0;
    float tmp_acc_z = 0.0;
    float tmp_mag_x = 0.0;
    float tmp_mag_y = 0.0;
    float tmp_mag_z = 0.0;
    int32_t tmp_latitude = 0;
    int32_t tmp_longitude = 0;
    while (1) {
        switch (state) {
            case (IDLE): {
                Heltec.display->clear();
                char buff[40];

                Heltec.display->drawString(0, 0, "IDLE");

                // if (xSemaphoreTake(mutex_gps, 0) == pdTRUE) {
                //     tmp_latitude = latitude;
                //     tmp_longitude = longitude;
                //     Serial.print(latitude);
                //     Serial.print(", ");
                //     Serial.print(longitude);
                //     xSemaphoreGive(mutex_gps);
                // }

                sprintf(buff, "Lat: %i", tmp_latitude);
                Heltec.display->drawString(0, 20, buff);

                sprintf(buff, "Long: %i", tmp_longitude);
                Heltec.display->drawString(0, 30, buff);

                if (xSemaphoreTake(mutex, 0) == pdTRUE) {
                    tmp_altitude = altitude;
                    xSemaphoreGive(mutex);
                }

                sprintf(buff, "Altitude: %.2f ft", tmp_altitude);
                Heltec.display->drawString(0, 40, buff);

                // sprintf(
                //     buff,
                //     "%i/%i/%i %i:%i:%i",
                //     myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(), myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());
                // Heltec.display->drawString(0, 30, buff);

                // sprintf(buff, "SkyNet IDLE: %i", counter);

                Heltec.display->display();  // display all the above display calls

                // Check if rocket should be armed
                if (check_if_armed()) {
                    // Serial.println("STATUS CHANGE - ARMED");
                    altitude_zero = tmp_altitude;
                    sprintf(buff, "Zero Altitude = %f", altitude_zero);
                    Heltec.display->clear();
                    Heltec.display->drawString(0, 0, "ARMED");
                    Heltec.display->drawString(0, 20, buff);
                    Heltec.display->display();
                    // Heltec.display->displayOff();
                    sendPacket("ARMD");
                    state = ARMED;
                } else {
                    if (millis() - lastSendTime > interval) {
                        lastSendTime = millis();
                        interval = random(2000) + 1000;  // 1-3 seconds    // Reset last send time
                        sendPacket("IDLE");
                    }
                }
                vTaskDelay(500 / portTICK_PERIOD_MS);
                break;
            }

            case (ARMED): {
                bool new_data = false;
                if (xSemaphoreTake(mutex, 0) == pdTRUE) {
                    tmp_temperature = temperature;
                    tmp_pressure = pressure;
                    tmp_altitude = std::abs(altitude - altitude_zero);
                    tmp_acc_x = acc_x;
                    tmp_acc_y = acc_y;
                    tmp_acc_z = acc_z;
                    tmp_gyro_x = gyro_x;
                    tmp_gyro_y = gyro_y;
                    tmp_gyro_z = gyro_z;
                    tmp_mag_x = mag_x;
                    tmp_mag_y = mag_y;
                    tmp_mag_z = mag_z;
                    xSemaphoreGive(mutex);
                    new_data = true;
                }

                if (new_data) {
                    // Send all sensor data over LoRa
                    LoRa.beginPacket();
                    LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST);
                    LoRa.write((uint8_t)0);
                    LoRa.write((uint8_t *)&tmp_temperature, 4);
                    LoRa.write((uint8_t *)&tmp_pressure, 4);
                    LoRa.write((uint8_t *)&tmp_altitude, 4);
                    LoRa.write((uint8_t *)&tmp_acc_x, 4);
                    LoRa.write((uint8_t *)&tmp_acc_y, 4);
                    LoRa.write((uint8_t *)&tmp_acc_z, 4);
                    LoRa.write((uint8_t *)&tmp_gyro_x, 4);
                    LoRa.write((uint8_t *)&tmp_gyro_y, 4);
                    LoRa.write((uint8_t *)&tmp_gyro_z, 4);
                    LoRa.write((uint8_t *)&tmp_mag_x, 4);
                    LoRa.write((uint8_t *)&tmp_mag_y, 4);
                    LoRa.write((uint8_t *)&tmp_mag_z, 4);
                    LoRa.endPacket();
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);

                break;
            }

            case (CLI): {
                constexpr int BUFF_SIZE = 50;
                char cmd_buff[BUFF_SIZE];
                Serial.print("> ");

                while (1) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    // while (not Serial.available());
                    if (Serial.available()) {
                        // read input from user and convert to a std::string
                        size_t bytes_read = Serial.readBytesUntil('\n', cmd_buff, BUFF_SIZE);
                        cmd_buff[bytes_read] = '\0';  // I think std::string wants the \0
                        std::string in_string = std::string(cmd_buff, std::strlen(cmd_buff));
                        in_string = utils::trim_string(in_string);  // trim whitespace
                        Serial.println(in_string.c_str());          // echo command back to user

                        // split the string by spaces into a vector<string>
                        std::vector<std::string> input_vec = utils::split_string(in_string, ' ');
                        std::string cmd = input_vec.at(0);  // first is the command itself

                        // ls: list contents of the root directory
                        if (cmd == "ls") {
                            fs::list_dir(SPIFFS, "/", 0);
                        }

                        // cat: print the contents of a file to the screen
                        else if (cmd == "cat") {
                            if (input_vec.size() == 2) {
                                std::string arg = input_vec.at(1);
                                arg.insert(0, "/");
                                fs::read_file(SPIFFS, arg.c_str());
                            } else {
                                Serial.println("No argument passed to cat command");
                            }
                        }

                        else if (cmd == "rm") {
                            if (input_vec.size() == 2) {
                                std::string arg = input_vec.at(1);
                                arg.insert(0, "/");
                                fs::delete_file(SPIFFS, arg.c_str());
                            } else {
                                Serial.println("No argument passed to rm command");
                            }
                        }

                        // help: print a help message
                        else if (cmd == "help") {
                            Serial.print(
                                "\n"
                                "------------------------\n"
                                " SkyNet Transmitter CLI \n"
                                "------------------------\n"
                                "---Available Commands---\n"
                                "ls: list all files\n"
                                "cat: print the contents of the specified file to the screen\n"
                                "rm: permanantly delete the specified file\n"
                                "help: print this help message to this screen\n\n");
                        }

                        else {
                            Serial.print("Unknown command ");
                            Serial.println(cmd.c_str());
                        }

                        Serial.print("> ");
                    }
                }
                break;
            }
        }
        // counter++;
    }
}

// read from sensors and update values
void sensor_task(void *params) {
    char data_buff[200];
    while (1) {
        if (state != CLI) {
            if (xSemaphoreTake(mutex, 0) == pdTRUE) {
                led_flag = 1 ^ led_flag;
                digitalWrite(LED_BUILTIN, led_flag);

                // Read pressure sensor
                if (!bmp.performReading()) {
                    return;
                }
                // Store BMP data
                temperature = bmp.temperature;
                pressure = bmp.pressure;
                float altitude_m = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                altitude = altitude_m * FEET_PER_METER;  // convert to feet

                // read IMU
                magnetometer->getEvent(&mag_event);
                gyroscope->getEvent(&gyro_event);
                accelerometer->getEvent(&accel_event);

                acc_x = accel_event.acceleration.x;
                acc_y = accel_event.acceleration.y;
                acc_z = accel_event.acceleration.z;
                gyro_x = gyro_event.gyro.x;
                gyro_y = gyro_event.gyro.y;
                gyro_z = gyro_event.gyro.z;
                mag_x = mag_event.magnetic.x;
                mag_y = mag_event.magnetic.y;
                mag_z = mag_event.magnetic.z;

                char data_fmt[] = "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n";
                sprintf(data_buff, data_fmt,
                        0,
                        temperature,
                        pressure,
                        altitude,
                        acc_x,
                        acc_y,
                        acc_z,
                        gyro_x,
                        gyro_y,
                        gyro_z,
                        mag_x,
                        mag_y,
                        mag_z);
                Serial.print(data_buff);

                xSemaphoreGive(mutex);

                if (state == IDLE) {
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                } else if (state == ARMED) {
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }

            } else {
                // do something else?
            }
        } else {
            // delete the task if in CLI mode
            vTaskDelete(NULL);  // UNTESTED
        }
    }
}

// get latitude and longitude from GPS
void gps_task(void *params) {
    while (true) {
        if (xSemaphoreTake(mutex_gps, 0) == pdTRUE) {
            latitude = myGPS.getLatitude();
            longitude = myGPS.getLongitude();
            xSemaphoreGive(mutex_gps);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void setup(void) {
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
    delay(1000);  // wait for board to be initialized
    LoRa.setSignalBandwidth(500E3);

    // CLI can only be enable on startup
    // pinMode(0,INPUT);
    // if (not digitalRead(0)) {
    //   state = CLI;
    // }

    // probably only need serial for CLI mode
    Serial.begin(115200);  // init serial for testing
    while (!Serial) {
        delay(10);
    }

    if (state == CLI) {
        Serial.setTimeout(10000);
    }

    Heltec.VextON();               // Enable 3.3V Vext
    I2CBUS.begin(13, 22, 100000);  // start I2C bus

    // Initialize GPS
    if (myGPS.begin(I2CBUS, 0x42, 1100, false) == false) {
        // Serial.println(F("Failed to find Ublox GPS"));
        Heltec.display->drawString(0, 20, "Failed to find Ublox GPS");
        Heltec.display->display();
        while (1) {
            ;
        }
    } else {
        // Serial.println(F("Booted GPS"));
        Heltec.display->drawString(0, 20, "Booted GPS");
        Heltec.display->display();
    }

    // Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.setI2COutput(COM_TYPE_UBX);
    // myGPS.enableDebugging();

    // Initialize BMP388 pressure sensor
    if (!bmp.begin_I2C(0x77, &I2CBUS)) {
        // Serial.println(F("Failed to boot BMP388"));
        Heltec.display->drawString(0, 30, "Failed to boot BMP388");
        Heltec.display->display();
        while (1) {
            ;
        }
    } else {
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
    if (!init_sensors(&I2CBUS)) {
        // Serial.println(F("Failed to find IMU sensors"));
        Heltec.display->drawString(0, 40, "Failed to find IMU sensors");
        Heltec.display->display();
        while (1) {
            delay(10);
        }
    } else {
        // Serial.println(F("Booted IMU sensors"));
        Heltec.display->drawString(0, 40, "Booted IMU sensors");
        Heltec.display->display();
    }

    setup_sensors();

    Wire.setClock(400000);  // 400KHz

    // Mount filesystem
    if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
        Serial.println("SPIFFS Mount Failed");
        return;
    }

    // declare mutexes
    mutex = xSemaphoreCreateMutex();
    mutex_gps = xSemaphoreCreateMutex();

    // create task for LoRa communication
    xTaskCreate(
        main_loop,    // function
        "Main loop",  // task name
        4096,         // stack size
        NULL,         // task parameters
        2,            // task priority
        NULL          // task handle
    );

    // Create a task for fast data logging to flash memory
    xTaskCreate(
        sensor_task,    // function
        "Sensor task",  // task name
        4096,           // stack size
        NULL,           // task parameters
        1,              // task priority
        NULL            // task handle
    );

    // Create a task for reading from the GPS
    // xTaskCreate(
    //     gps_task,    // function
    //     "GPS task",  // task name
    //     4096,        // stack size
    //     NULL,        // task parameters
    //     1,           // task priority
    //     NULL         // task handle
    // );
}

void loop() {}
