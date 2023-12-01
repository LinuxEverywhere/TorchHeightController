# TorchHeightController
EPS32 based THC that reads plasma cutter voltage and send Up and Down signals to Plasma Torch Actuator to adjust voltage to target value.

* How to Build Video: https://www.youtube.com/watch?v=LxGfTt5eavE
* Build Information: https://openbuilds.com/builds/standalone-torch-height-controller-for-cnc-plasma-cutting.9571/

## Description
Aim is to create a low cost and easy to use Torch Height Controller with off the shelf parts, simple easy to read code and as little electronics tinkering as possible.
This is a standalone type of THC so it requires an actuator to move the torch independent of the CNC machines controller. 
If you interface this THC with your CNC Machine please post how you did it!

The unknown we are trying to solve for here is the Torch Height from the Workpiece measurement. 
It's important for a plasma arc to be stable and be a set height from the workpiece to be cut. 
The main reason for this is the plasma arc will cut a bevel on the side walls if the height is not set right or crash into the workpiece...
This is because the plasma arc is not like a laser with straight edges but more like an egg. 
Making the problem worse is the fact the metal can warp and contort when a hot plasma arc cuts into it. 
Using the Arc Voltage is a good way to estimate the distance to the workpiece from the torch head. 

The proportional correlation is the longer the arc the higher the voltage. 
So, we can measure the plasma voltage and feed that into a PID Algorithm to calculate the torch height to change the voltage to a setpoint. 
It is unwise to measure the Arc Voltage Directly off the plasma torch because the levels there can be deadly. 
Most CNC ready Plasma Cutters on the market have 50:1 - 16:1 arc voltage dividers built right into the machine. 
If yours doesn't have this then you will need to do surgery and add a voltage divider circuit to your plasma cutter... 
Check the Technical Specs of your plasma cutter: 
Don't Die.

## Hardware
* 2 x Esp32 dev board 
* Nextion HMI Screen: NX4832T035_011
* tb6600 stepper driver
* Power: 24v plama cutter for remote; 5V common rail for THC/main controller.
* PC diagnostic: USB A - USB B
* MicroSD card: 8gb

## Donate:
* https://www.patreon.com/HaleDesign

## More info:
* Fusion 360 Design File: https://a360.co/30zrSRM
* About Hale Design Tech: http://hdt.xyz

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

## Download needed software
 * Auduino IDE: https://www.arduino.cc/
 * FastPID Library: Install via Manage Libraries in Arduino IDE
 * EasyNextionLibrary: Install via Manage Libraries in Arduino IDE
 * AccelStepper: Install via Manage Libraries in Arduino IDE
 * Nextion Editor(optional): https://nextion.tech 

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

## Uploading HaleDesign_-_Torch_Height_Controller.ino Arduino sketch
Follow these steps to upload your sketch:
* Connect your Arduino using the USB cable. 
* Choose Tools→Board→Arduino Uno to find your board in the Arduino menu. 
* Choose the correct serial port for your board. 
* Click the Upload button.

## Main Process Code 
This code is designed to run as fast as possible. It uses a while() loop to focus on Calulations and movements when voltage input is over the threshold.

process(){
...
}

