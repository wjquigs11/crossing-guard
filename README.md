# **Diamond Crossing Controller**

This project implements an automated controller for a model railroad diamond crossing using an ESP32 microcontroller. It manages signals, track power, and train detection to safely control traffic through the crossing.

The MCU is connected to 5 VL53L1X distance sensors embedded in the track. When a train passes over the sensor, the reading (compared to a moving average) drops. There is a simple state machine for the crossing. All 4 signals start at yellow. When a train enters the zone near the crossing, its sensor goes to green and all others go to red. If a train passes a sensor and its corresponding signal is red, the system triggers a relay to cut power to the second train's section of track. If either train is using DCC instead of DC, the system instead sends a "speed 0" command to the second train. When the first train passes over the sensor on the other side of the crossing, the crossing enters the "clearing" state. When the second sensor detects no more train, the crossing enters the "clear" state and the second train can proceed.

The crossing has 4 directions, but one direction immediately splits into two tracks, hence the need for 5 sensors. Since each of the 4 signals has 3 LEDs, I consume a lot of GPIOs, so I'm using a Sparkfun I2C Mux and Adafruit MCP23017 GPIO expander to get enough pins.

TBD: the diamond crossing MCU forwards sensor triggers to another MCU ("turnout") to automatically switch tracks, depending on the location of the trains. It's a work in progress.

## Configuration
The system uses ESP32's preferences to store:

## WiFi settings
DCC-EX command station hostname
Locomotive IDs for North-South and East-West routes

The system supports three crossing states:
Clear - All signals yellow, trains may proceed
Occupied - One train in crossing, signals set accordingly
Clearing - Train exiting crossing, waiting for complete clearance

## Signal Logic
Yellow signals indicate track is clear to proceed
Green signal shows for train currently cleared through crossing
Red signals stop opposing traffic

Automatic detection prevents conflicting movements

## Safety Features
Track power control via relays for non-DCC locomotives
DCC brake commands for DCC-equipped locomotives
Sensor calibration at startup
Multiple sensor verification for train position

## Serial Commands
wifi - Toggle WiFi enable/disable (requires reboot)
restart - Restart the controller

Web interface available for monitoring and configuration commands:
## Webserial Commands
? - Display list of available commands
format - Format the SPIFFS filesystem
restart - Restart the ESP32
ls - List files in SPIFFS storage
scan - Scan I2C bus
status - Show uptime and system status
### Configuration Commands
hostname [name] - Get or set device hostname
wificonfig - Display WiFi configuration (SSID, IP, MAC address)
log - Toggle serial logging on/off
teleplot - Toggle teleplot output on/off
### Sensor Commands
sensor [threshold] - Show sensor readings or set threshold percentage for all sensors
calibrate - Run sensor calibration routine
state - Print signal state information
### Train Control Commands
stop [i/o] - Stop trains (inner loop, outer loop, or both if no parameter)
start [i/o] - Start trains (inner loop, outer loop, or both if no parameter)
relay - Show relay status for inner/mountain and outer/flat loops
speed <loco> - Get speed of specified locomotive
brake <loco> - Apply brakes to specified locomotive
resume <loco> - Resume locomotive operation
### Locomotive Assignment Commands
loco ns <id> - Set North-South locomotive ID
loco ew <id> - Set East-West locomotive ID
System Control Commands
disable - Disable relay control
enable - Enable relay control and set signals to yellow
<> - Pass through commands to DCC-EX command station

Each command is processed through the WebSerial interface and can be entered through the web console when connected to the device.

## Installation
Configure hardware connections according to pin definitions
Set up WiFi credentials
Configure DCC-EX command station if using DCC
Upload code to ESP32

