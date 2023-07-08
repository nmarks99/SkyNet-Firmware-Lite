# SkyNet Firmware (Nick's Fork)
*This fork introduces some potential improvements to the SkyNet embedded firmware
and should be integrated into the main SkyNet project (https://github.com/NUSTARS/SkyNet-Hardware)
later on. This code was flown in SkyNet's first and only test flight in June 2023.*

First, take a look at the GitHub Wiki Charles Zhou made [here](https://github.com/NUSTARS/SkyNet/wiki).
It gives a pretty good summary of the project, however my fork here is somewhat difference, hence 
this README. I will go ahead and give my own summary of SkyNet here as well.

## Hardware
SkyNet is a hardware/software system designed to stream real time data from a rocket during flight
to a groundstation computer using [LoRa](https://en.wikipedia.org/wiki/LoRa) radio. I will not
discuss LoRa much here, however all you really need to know is that LoRa has a pretty solid range
(we had connection the entire time during our test flight which flew a few thousand feet) but with
our current setup, we haven't gotten it to be able to send data any faster than about 2Hz.

The hardware is composed of a [Heltec WiFi LoRa 32 V2](https://heltec.org/project/wifi-lora-32/)
which is an ESP32 based microcontroller, and a [BerryGPS-IMU V4](https://ozzmaker.com/product/berrygps-imu/).
It also has a time-of-flight sensor which was originally intended to be used to detect separation
of the rocket but I think alternative methods should be considered. These components are mounted a
custom PCB and make up the transmitter board that goes in the rocket. Another Heltec board is used as
the receiver board and plugs into a laptop on the ground.

## Software
The main goal of the software originally was to simply read the sensors on the BerryGPS-IMU board
over I2C and send them to the groundstation board. The groundstation board recieves the data sent
via LoRa and then sends it to the laptop over a USB cable using serial. All the code on the
transmitter and reciever boards is written in Arduino (C++) so in practice, what is happening 
here is the transmitter does something like `LoRa.write(data)`, and the reciever board does
something like `data = LoRa.read()`, and then `Serial.print(data)`. So as you can imagine,
you can read the data from the serial port on your computer using basically any programming
language you want and display it however you want. Our solution so far has been to make a 
cross-platform JavaScript application: [SkyNet Groundstation](https://github.com/NUSTARS/SkyNet/)
however it needs some work. The key thing to note here is that at the most basic level, all the 
groundstation is doing is reading data over the serial port and displaying it. We have struggled
to find people with JavaScript experience (we picked JS in the first place because it can make
very pretty GUIs) so if you can find some other tool or what to write something up in a different
language to display the data, have at it.

As I said, the above was the *original* goal. Later on we determined that there are some additional
features that we would like plus some things that we don't really need. We don't really need to
send *all* the data, which includes 9-axis IMU, pressure, altitude, temperature, and GPS.
During flight, we probably only really care about altitude and GPS, however it would be nice
to save the rest of the data onboard somehow so we can look at it later. The code in this repo
attempts to solve this issue.

The ESP32 is a dual-core chip, so it can run two processes in parallel. Furthermore, ESP32
has great FreeRTOS support which enables many *tasks* to run at once, even more than one per core
and all the timing is handled under the hood by FreeRTOS. What this means in practice here is that
you can have mulitple tasks (which are simply implemented as functions) running at different
rates. What I have attempted to implement in this code here involves 3 FreeRTOS tasks which
update variables which are implemented as FreeRTOS semaphores. Semaphores are like mutexes
and I'm not totally sure of the distinction, however the purpose is to enable different 
threads to access shared data without running into race conditions (when two processes attempt
to access the same piece of memory at the same time). 

One task is responsible for sending the data over LoRa to the groundstation. The speed bottleneck
on this task is the LoRa transmission rate, which for the data we are sending now is seemingly 2Hz. 
Effort should be made to try and speed this up. We also do not need to be sending as much data
as we are as I mentioned before. Another task is used to read the GPS. This doesn't need to be
all that fast and is only really useful when the rocket has landed anyway. The last task is used
to read the rest of the sensor data over I2C very fast (comparatively, so maybe 50Hz?) and save it
to onboard flash memory. This is fully implemented in the code using SPIFFS, however I have found
that once the size of the file gets large, the saving of the data slows down significantly, and
worse sometimes it crashes. During our test flight, we did not save the data to onboard flash.
I believe if instead of onboard flash, we used an SD card module, saving the data could be faster,
or alternatively you could look into some alteratnive method for saving data other than SPIFFS.

To obtain the saved data, I have wrote a basic CLI mode for the transmitter board. When the state
is switched to CLI, you can open a serial monitor and send "help" to see the available commands.
More commands can and should be implemented. Currently there is no real way to switch the state
of the transmitter into CLI mode without reflashing the board. Two ways I would recommend
implmenting this could be either a physical switch that changes the state variable to CLI,
or a command sent over serial that does it. Either way, this definitely should be implemented
ASAP. In general, a user should *never* need to reflash the board to use any of the features.

## Dev Environment
Since many people who may start working on this project may not have much experience with 
embedded software, Arduino, C++, etc. I figured I'd add this section.

Although the Arduino IDE could work for building and flashing code to the boards, it kinda
sucks so we use [PlatformIO](https://docs.platformio.org/en/latest/what-is-platformio.html).
Most people use it as a vscode extension, although if you prefer it is also available as a 
CLI tool (PlatformIO Core). There are lots of tutorials and examples online of how to use
platformio with Arduino if you need help.



