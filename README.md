# TorchHeightController
EPS32 based THC that reads plasma cutter voltage and send Up and Down signals to Plasma Torch Actuator to adjust voltage to target value.

* Forked from: https://openbuilds.com/builds/standalone-torch-height-controller-for-cnc-plasma-cutting.9571/

## Description
Aim is to create a low cost and easy to use Torch Height Controller with off the shelf parts, simple easy to read code and as little electronics tinkering as possible.
This is a standalone type of THC so it requires an actuator to move the torch independent of the CNC machines controller.

The unknown we are trying to solve for here is the Torch Height from the Workpiece measurement.
It's important for a plasma arc to be stable and be a set height from the workpiece to be cut.
The main reason for this is the plasma arc will cut a bevel on the side walls if the height is not set right or crash into the workpiece...
This is because the plasma arc is not like a laser with straight edges but more like an egg.
Making the problem worse is the fact the metal can warp and contort when a hot plasma arc cuts into it.
Using the Arc Voltage is a good way to estimate the distance to the workpiece from the torch head.

The proportional correlation is the longer the arc the higher the voltage.
So, we can measure the plasma voltage and feed that into a PID Algorithm to calculate the torch height to change the voltage to a setpoint.

For Plasma cutters without an isolated 50:1 volt cnc output.

Hardware:
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
2 x Esp32 dev board
Nextion HMI Screen: NX4832T035_011
tb6600 stepper driver
Power: 5V common rail with cnc controller 3.3v tap for remote

3rd Party Software:
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
FastPID Library
by Mike Matera
The FastPIDPID Library is available in the Arduino IDE Library Manager
Latest version on GitHub: https://github.com/mike-matera/FastPID
Version Used: v1.3.1

EasyNextionLibrary
by Athanasios Seitanis
contact: eithagta@gmail.com
The EasyNextionLibrary is available in the Arduino IDE Library Manager
From: https://github.com/Seithan/EasyNextionLibrary
Version Used: v1.0.6

AccelStepper
by Mike McMauley
The AccelStepper is available in the Arduino IDE Library Manager
From: http://www.airspayce.com/mikem/arduino/AccelStepper/
Version Used: v1.64.0

BluetoothSerial
By Evandro Copercini
Version Used: 2.0.0
Preferences
ByHristo Gochkov
Version Used: 2.0.0
From: Additional Boards Manager URL: https://dl.espressif.com/dl/package_esp32_index.json

## Hardware
* 2 x Esp32 dev board
* Nextion HMI Screen: NX4832T035_011
* tb6600 stepper driver
* Power: 3.3v plasma cutter tap for remote; 5V common rail for THC and main controller.

## 3rd Party software used
 * Auduino IDE
 * FastPID Library
 * EasyNextionLibrary
 * AccelStepper
 * PreferencesLibrary
 * Nextion Editor

# Setup and Configuration
The User Interface is a Nextion standalone HMI Screen that can adjust most setting and Save tro EEPROM without update Arduino Code. The beauty of use this screen is it handles all the heavy UI events leaving the arduino free to calulate as fast as possible. The Arduino talks with the Nextion Screen via Serial Connection and updates all values. As the user triggers events on the nextion screen the they are reported to the Arduino borad for processing and any calulations needed.

## Main Features
 * Input Voltage Sampling
 * steps per millimeter
 * Delay Time
 * Threshold
 * Aggressive PID Tuning
 * Conservative PID Tuning
 * PID calulation for move direct and distance
 * Max distances Actuator can adjuest
 * Steps and Direction output
 * Position and Voltage reporting

## Upload TCH GUI V.x.x.TFT project file to Nextion Screen
* Required: Nextion HMI Screen NX4832T035_011
if you don't have this screen then you will have to rebuild the project for your screen using the Nextion Editor and included project files.
if you do this please post your Project and TFT file!

Nextion’s microSD slot is primarily used to upload a TFT project file.
Not all microSD cards are made for use with embedded devices especially newer microSD cards made for cameras, etc.
Class 10 HC 8GB to 32GB cards have had good success.

Ensure
* microSD card formatted as FAT32 under Windows
* microSD card is less than 32 GB
* only 1 TFT project file exists on the microSD card
* Nextion device is powered off  before inserting microSD card
* insert the microSD card containing TFT project file
* power on Nextion with recommended clean power as per Datasheets
* Nextion device is undisturbed while uploading
* after upload “successed” indication power off Nextion
* after Nextion device is powered off  then remove microSD card
* power on Nextion with recommended clean power as per Datasheets
* if new version of Nextion Editor, allow time for firmware to update
* wait for uploaded project to begin running on Nextion device

If  microSD upload is unsuccessful
* Reformat the microSD card –  ensuring FAT32 under Windows, and try.
* use a different microSD card (HC, Class 10, under 32GB), and try.
* compile a blank HMI project and try again.
* microSD card is not the only method to upload project – try via USB to TTL.

## Main Process Code
This code is designed to run as fast as possible. It uses a while() loop to focus on Calulations and movements when voltage input is over the threshold.

process(){
...
}
