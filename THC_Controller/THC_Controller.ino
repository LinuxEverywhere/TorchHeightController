/*
    Torch Height Controller

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

  Software Version: 2.0.1.alpha

  Aim:
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  "THC_Remote" sends torch voltage to "THC_Controller", which moves the torch to match the voltage set point.
  Designed for Plasma cutters without cnc isolated 50:1 volt output.

  Hardware:
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  1 x Esp32 dev board
  Nextion HMI Screen: NX4832T035
  tb6600 stepper driver
  200:3.3 stepdown voltage circuit (isolated)
  Power: 5V common rail with cnc controller with hotswapping stepper outputs between contollers in hardware

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

  Input:
  Arc Voltage, conservative P I D parameters, aggressive P I D parameters, Gap(amount away from setpoint for Agg/Con PID Settings), Arc Voltage Setpoint, ArcStablizeDelay, and Z axes bountray limits.

*/
#include <FastPID.h>              // Include PID Library
#include <EasyNextionLibrary.h>   // Include EasyNextionLibrary
#include <AccelStepper.h>
#include <BluetoothSerial.h>
#include <Preferences.h>

// the variables to be using be the code below

EasyNex THCNex(Serial); // Create an object of EasyNex class with the name < TCHNex >
Preferences Saved;

//#define LED_PIN 2
#define PLASMA_ON_PIN 4
#define STEP_PIN 23      // Direction
#define DIR_PIN 22       // Step

unsigned long m, lastreadTime = 0;

//#define USE_NAME // Comment this to use MAC address instead of a slaveName

const char *pin = "43f4"; // Change this to reflect the pin expected by the real slave BT device

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
  String slaveName = "THC_Remote"; // Change this to reflect the real name of your slave BT device
#else
  String MACadd = "AA:BB:CC:11:22:33"; // This only for printing
  uint8_t address[6]  = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33}; // Change this to reflect real MAC address of your slave BT device
#endif

String myName = "THC_Contoller";
bool connected;

// Define a stepper driver and the pins it will use
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

//Define Global Variables
//double Input = 0;
double targetInput;
double scale;
int threshold;
int currentGap, gap;
uint32_t oldDelay;
uint32_t arcStabilizeDelay;
int SetPoint,Input,CalibrationOffset = 0;
bool debug = false;

int defaultSetpoint = 10900;

int SetpointPage1 = 0;
int SetpointPage2 = 0;
int SetpointPage3 = 0;
int SetpointPage4 = 0;
int SetpointPage5 = 0;
int SetpointPage6 = 0;

int CurrentPageNumber = 0;
int SavedPage = 0;

//movement
int steps_per_mm = 160;
float pos = 0;
float adjpos = 0;
int minPos = -(16 * steps_per_mm);
int maxPos = (16 * steps_per_mm);
int moveAmt = 0;
uint8_t output = 0;

//Specify the links and initial tuning parameters
float aggKp = 0.175, aggKi = 0.1, aggKd = 0.1;
float Kp = 0.075, Ki = 0.01, Kd = 0.01;
float Hz = 8;
int output_bits = 16;
bool output_signed = true;
bool alreadySetColor = false;

FastPID THCPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

void OpenSaved(bool open = true) {
  if (open) {
    Saved.begin("Saved",false);
  } else {
    Saved.end();
  }
}

// the setup function runs once when you press reset or power the board
void setup() {

  SerialBT.begin(myName); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", myName.c_str());
  //Serial.printf("The device with name \"%s\" and MAC address %s is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str(), SerialBT.getMacString()); // Use this after the MAC method is implemented
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
  // Initialize digital pin LED_BUILTIN as an output.
  //This is used to enable the MKS driver board. Plus it flashes and flashes are cool.
  pinMode(PLASMA_ON_PIN, INPUT);

  // Begin the object with a baud rate of 9600
  THCNex.begin();  // If no parameter was given in the begin(), the default baud rate of 9600 will be used
  if (debug) { Serial.begin(115200); };//for debugging via usb.
//  while(!Serial);
  OpenSaved();
  //initialize the variables we're linked to
  // Load EEPROM Addresses for Setpoints or set defaults
  SetpointPage1 = Saved.getInt("Page1",defaultSetpoint);

  SetpointPage2 = Saved.getInt("Page2",defaultSetpoint);

  SetpointPage3 = Saved.getInt("Page3",defaultSetpoint);

  SetpointPage4 = Saved.getInt("Page4",defaultSetpoint);

  SetpointPage5 = Saved.getInt("Page5",defaultSetpoint);

  SetpointPage6 = Saved.getInt("Page6",defaultSetpoint);

  scale = Saved.getDouble("Scale",0);
  if (scale == 0) {
    scale = 1;
  }

  gap = Saved.getInt("Gap",500);

  threshold = Saved.getInt("Threshold",4000);

  arcStabilizeDelay = Saved.getInt("Delay",500);

  steps_per_mm = Saved.getInt("Steps",160);

  maxPos = Saved.getInt("Maxpos",maxPos);

  minPos = Saved.getInt("Minpos",minPos);

  aggKp = Saved.getFloat("AP",0.175);

  aggKi = Saved.getFloat("AI",0.1);

  aggKd = Saved.getFloat("AD",0.1);

  Kp = Saved.getFloat("CP",0.075);

  Ki = Saved.getFloat("CI",0.01);

  Kd = Saved.getFloat("CD",0.01);

  CalibrationOffset = Saved.getInt("Calibrate",0);

  OpenSaved(false);

  // Wait for Nextion Screen to bootup
  delay(2500);
  THCNex.writeNum("CustomSetPoint.val", SetpointPage1);
  THCNex.writeNum("CustomSetPoint.val", SetpointPage1); //Make sure it set
  THCNex.writeNum("CustomSetPoint.val", SetpointPage1); //One more time
  SetPoint = SetpointPage1;

  //Setup Stepper Driver
  stepper.setMaxSpeed(1600); //thru experimentation I found these values to work... Change for your setup.
  stepper.setAcceleration(96);
}

// the loop function runs over and over again forever
void loop()
{
  THCNex.NextionListen();

  process(); //This is the main method of the application it calulates position and move steps if Input Voltage is over threshold.
  if (CurrentPageNumber <= 6 || CurrentPageNumber == 11) {
    report();
  }
}

void process() //Calulates position and move steps
{
  oldDelay = micros();
  if (!SerialBT.available()) {
    SerialBT.println("get");
    delay(400);
  }
  if (SerialBT.available()) {
    Input = (int)SerialBT.read() + CalibrationOffset;
  } else { Input = SetPoint; }
  while (Input > (threshold + CalibrationOffset) && digitalRead(PLASMA_ON_PIN) == HIGH) //Only move if cutting by checking for voltage above a threshold level
  {
    if (micros() - oldDelay >= arcStabilizeDelay) //wait for arc to stabilize tipically 100-300ms
    {
      if (SerialBT.available()) {
        Input = (int)SerialBT.read() + CalibrationOffset;
      } else { SerialBT.println("get"); } //get new plasma arc voltage and convert to millivolts

      currentGap = abs(SetPoint - Input); //distance away from setpoint
      if (currentGap < gap) {
        THCPID.setCoefficients(Kp, Ki, Kd, Hz); //we're close to setpoint, use conservative tuning parameters
      }
      else {
        THCPID.setCoefficients(aggKp, aggKi, aggKd, Hz); //we're far from setpoint, use aggressive tuning parameters
      }

      if (SetPoint > Input)
      {
        targetInput = Input - SetPoint;
        output = THCPID.step(SetPoint, targetInput);
        pos = pos + output;
      }
      else
      {
        targetInput = SetPoint - Input;
        output = THCPID.step(SetPoint, targetInput);
        pos = pos - output;
      }

      //Validate move is within range
      if (pos >= maxPos) {
        pos = maxPos;
      }
      if (pos <= minPos) {
        pos = minPos;
      }

      //do move
      stepper.moveTo(pos);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }

      report(); //report plasma voltage and position
      //format();
    }
  }
  //after cut reset height
  pos = 0;
  //do move
  stepper.moveTo(pos);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

void trigger0() //Set last page used on startup loaded event
{
  //not used because bug with Nextion screen not updating screen loaded event.
  //THCNex.writeNum("dp", SavedPage);
}

void trigger1() //read customsetpoint on page loaded event
{
  CurrentPageNumber = THCNex.readNumber("dp");
  SetPoint = THCNex.readNumber("CustomSetPoint.val");
  if (CurrentPageNumber != 777777 && SetPoint != 777777)
  {
    switch (CurrentPageNumber) {
      case 1:
        SetPoint = SetpointPage1; //write a few times to make sure... nextion screen has a nasty habbat of ignoring update commands on boot.
        THCNex.writeNum("CustomSetPoint.val", SetpointPage1);
        THCNex.writeNum("CustomSlide.val", SetpointPage1);
        THCNex.writeNum("CustomSetPoint.val", SetpointPage1);
        THCNex.writeNum("CustomSlide.val", SetpointPage1);
        THCNex.writeNum("CustomSetPoint.val", SetpointPage1);
        THCNex.writeNum("CustomSlide.val", SetpointPage1);
        break;
      case 2:
        SetPoint = SetpointPage2;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage2);
        THCNex.writeNum("CustomSlide.val", SetpointPage2);
        break;
      case 3:
        SetPoint = SetpointPage3;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage3);
        THCNex.writeNum("CustomSlide.val", SetpointPage3);
        break;
      case 4:
        SetPoint = SetpointPage4;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage4);
        THCNex.writeNum("CustomSlide.val", SetpointPage4);
        break;
      case 5:
        SetPoint = SetpointPage5;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage5);
        THCNex.writeNum("CustomSlide.val", SetpointPage5);
        break;
      case 6:
        SetPoint = SetpointPage6;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage6);
        THCNex.writeNum("CustomSlide.val", SetpointPage6);
        break;
      default:
        break;
    }
  }
}
void trigger2() //Save customsetpoints on end touch event
{
  CurrentPageNumber = THCNex.readNumber("dp");
  SetPoint = THCNex.readNumber("CustomSetPoint.val");
  if (CurrentPageNumber != 777777 && SetPoint != 777777)
  {
    OpenSaved();
    switch (CurrentPageNumber) {
      case 1:
        SetpointPage1 = SetPoint;
        Saved.putInt("Page1", SetpointPage1);
        break;
      case 2:
        SetpointPage2 = SetPoint;
        Saved.putInt("Page2", SetpointPage2);
        break;
      case 3:
        SetpointPage3 = SetPoint;
        Saved.putInt("Page3", SetpointPage3);
        break;
      case 4:
        SetpointPage4 = SetPoint;
        Saved.putInt("Page4", SetpointPage4);
        break;
      case 5:
        SetpointPage5 = SetPoint;
        Saved.putInt("Page5", SetpointPage5);
        break;
      case 6:
        SetpointPage6 = SetPoint;
        Saved.putInt("Page6", SetpointPage6);
        break;
      default:
        break;
    }
    OpenSaved(false);
  }
}
void trigger3() //Move motor up
{
  pos = pos + (scale * steps_per_mm);
  stepper.moveTo(pos);
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  THCNex.writeNum("x2.val", (int)(pos / 2));
}
void trigger4() //Move motor down
{
  pos = pos - (scale * steps_per_mm);
  stepper.moveTo(pos);
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  THCNex.writeNum("x2.val", (int)(pos / 2));
}
void trigger5() //Increase allowable down movement range
{
  minPos = minPos + (scale * steps_per_mm);
  THCNex.writeNum("x1.val", (int)(minPos / 2));
  OpenSaved();
  Saved.putInt("Minpos", minPos);
  OpenSaved(false);
}
void trigger6() //Decrease allowable up movement range
{ minPos = minPos - (scale * steps_per_mm);
  THCNex.writeNum("x1.val", (int)(minPos / 2));
  OpenSaved();
  Saved.putInt("Minpos", (int)minPos);
  OpenSaved(false);
}
void trigger7() //Increase allowable up movement range
{
  maxPos = maxPos + (scale * steps_per_mm);
  THCNex.writeNum("x0.val", (int)(maxPos / 2));
  OpenSaved();
  Saved.putInt("Maxpos", (int)maxPos);
  OpenSaved(false);

}
void trigger8() //Decrease allowable down movement range
{
  maxPos = maxPos - (scale * steps_per_mm);
  THCNex.writeNum("x0.val", (int)(maxPos / 2));
  OpenSaved();
  Saved.putInt("Maxpos", (int)maxPos);
  OpenSaved(false);
}

void trigger9() //Increase voltage gap between aggressive and normal targeting
{
  gap = gap + (scale * 100);
  THCNex.writeNum("x2.val", (int)(gap));
  OpenSaved();
  Saved.putInt("Gap", (int)gap);
  OpenSaved(false);
}

void trigger10() //Decrease voltage gap between aggressive and normal targeting
{
  gap = gap - (scale * 100);
  THCNex.writeNum("x2.val", (int)(gap));
  OpenSaved();
  Saved.putInt("Gap", (int)gap);
  OpenSaved(false);
}
void trigger11() //Increase voltage reading threshold for calculating movements
{
  threshold = threshold + (scale * 100);
  THCNex.writeNum("x1.val", (int)(threshold));
  OpenSaved();
  Saved.putInt("Threshold", (int)threshold);
  OpenSaved(false);
}
void trigger12() //Decrease voltage reading threshold for calculating movements
{
  threshold = threshold - (scale * 100);
  THCNex.writeNum("x1.val", (int)(threshold));
  OpenSaved();
  Saved.putInt("Threshold", (int)threshold);
  OpenSaved(false);
}
void trigger13() //Increase delay before calculating movements
{
  arcStabilizeDelay = arcStabilizeDelay + (scale * 100);
  THCNex.writeNum("x0.val", (int)(arcStabilizeDelay / 10));
  OpenSaved();
  Saved.putInt("Delay", (int)arcStabilizeDelay);
  OpenSaved(false);
}
void trigger14() //Decrease delay before calculating movements
{
  arcStabilizeDelay = arcStabilizeDelay - (scale * 100);
  THCNex.writeNum("x0.val", (int)(arcStabilizeDelay / 10));
  OpenSaved();
  Saved.putInt("Delay", (int)arcStabilizeDelay);
  OpenSaved(false);
}
void trigger15() //Increase steps per millimeter
{
  steps_per_mm = steps_per_mm + scale;
  THCNex.writeNum("x3.val", (int)(100 * steps_per_mm));
  OpenSaved();
  Saved.putInt("Steps", (int)steps_per_mm);
  OpenSaved(false);
}
void trigger16() //Decrease steps per millimeter
{
  steps_per_mm = steps_per_mm - scale;
  THCNex.writeNum("x3.val", (int)(100 * steps_per_mm));
  OpenSaved();
  Saved.putInt("Steps", (int)steps_per_mm);
  OpenSaved(false);
}
void trigger17() //Increase Aggressive P Parameter
{
  aggKp = aggKp + scale * 0.01;
  THCNex.writeNum("x2.val", (int)(1000 * aggKp));
  OpenSaved();
  Saved.putFloat("AP", (float)aggKp);
  OpenSaved(false);
}
void trigger18() //Decrease Aggressive P Parameter
{
  aggKp = aggKp - scale * 0.01;
  THCNex.writeNum("x2.val", (int)(1000 * aggKp));
  OpenSaved();
  Saved.putFloat("AP", (float)aggKp);
  OpenSaved(false);
}
void trigger19() //Increase Aggressive I Parameter
{
  aggKi = aggKi + scale * 0.01;
  THCNex.writeNum("x1.val", (int)(1000 * aggKi));
  OpenSaved();
  Saved.putFloat("AI", (float)aggKi);
  OpenSaved(false);
}
void trigger20() //Decrease Aggressive I Parameter
{
  aggKi = aggKi - scale * 0.01;
  THCNex.writeNum("x1.val", (int)(1000 * aggKi));
  OpenSaved();
  Saved.putFloat("AI", (float)aggKi);
  OpenSaved(false);
}
void trigger21() //Increase Aggressive D Parameter
{
  aggKd = aggKd + scale * 0.01;
  THCNex.writeNum("x0.val", (int)(1000 * aggKd));
  OpenSaved();
  Saved.putFloat("AD", (float)aggKd);
  OpenSaved(false);
}
void trigger22() //Decrease Aggressive D Parameter
{
  aggKd = aggKd - scale * 0.01;
  THCNex.writeNum("x0.val", (int)(1000 * aggKd));
  OpenSaved();
  Saved.putFloat("AD", (float)aggKd);
  OpenSaved(false);
}
void trigger23() //Increase Conservative P Parameter
{
  Kp = Kp + scale * 0.01;
  THCNex.writeNum("x2.val", (int)(1000 * Kp));
  OpenSaved();
  Saved.putFloat("CP", (float)Kp);
  OpenSaved(false);
}
void trigger24() //Decrease Conservative P Parameter
{
  Kp = Kp - scale * 0.01;
  THCNex.writeNum("x2.val", (int)(1000 * Kp));
  OpenSaved();
  Saved.putFloat("CP", (float)Kp);
  OpenSaved(false);
}
void trigger25() //Increase Conservative I Parameter
{
  Ki = Ki + scale * 0.01;
  THCNex.writeNum("x1.val", (int)(1000 * Ki));
  OpenSaved();
  Saved.putFloat("CI", (float)Ki);
  OpenSaved(false);
}
void trigger26() //Decrease Conservative I Parameter
{
  Ki = Ki - scale * 0.01;
  THCNex.writeNum("x1.val", (int)(1000 * Ki));
  OpenSaved();
  Saved.putFloat("CI", (float)Ki);
  OpenSaved(false);
}
void trigger27() //Increase Conservative D Parameter
{
  Kd = Kd + scale * 0.01;
  THCNex.writeNum("x0.val", (int)(1000 * Kd));
  OpenSaved();
  Saved.putFloat("CD", (float)Kd);
  OpenSaved(false);
}
void trigger28() //Decrease Conservative D Parameter
{
  Kd = Kd - scale * 0.01;
  THCNex.writeNum("x0.val", (int)(1000 * Kd));
  OpenSaved();
  Saved.putFloat("CD", (float)Kd);
  OpenSaved(false);
}
void trigger29() //load movement page settings
{
  if (scale == 0.1)
  {
    THCNex.writeNum("bt0.val", 1);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 1.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 1);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 10.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 1);
  }
  THCNex.writeNum("x2.val", (int)(pos / 2));
  THCNex.writeNum("x0.val", (int)(maxPos / 2));
  THCNex.writeNum("x1.val", (int)(minPos / 2));
}
void trigger30() //Load default page settings
{
  if (scale == 0.1)
  {
    THCNex.writeNum("bt0.val", 1);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 1.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 1);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 10.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 1);
  }
  THCNex.writeNum("x2.val", (int)(gap));
  THCNex.writeNum("x1.val", (int)(threshold));
  THCNex.writeNum("x0.val", (int)(arcStabilizeDelay / 10));
  THCNex.writeNum("x3.val", (int)(100 * steps_per_mm));
}
void trigger31() //Save Scale on end touch event
{
  if (THCNex.readNumber("bt0.val") == 1)
  {
    scale = 0.1;
  }
  if (THCNex.readNumber("bt1.val") == 1)
  {
    scale = 1;
  }
  if (THCNex.readNumber("bt2.val") == 1)
  {
    scale = 10;
  }
}
void trigger32() //Load Aggressive PID settings
{
  if (scale == 0.1)
  {
    THCNex.writeNum("bt0.val", 1);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 1.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 1);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 10.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 1);
  }
  THCNex.writeNum("x2.val", (int)(1000 * aggKp));
  THCNex.writeNum("x1.val", (int)(1000 * aggKi));
  THCNex.writeNum("x0.val", (int)(1000 * aggKd));
}
void trigger33() //Load Conservative PID Settings
{ if (scale == 0.1)
  {
    THCNex.writeNum("bt0.val", 1);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 1.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 1);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 10.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 1);
  }
  THCNex.writeNum("x2.val", (int)(1000 * Kp));
  THCNex.writeNum("x1.val", (int)(1000 * Ki));
  THCNex.writeNum("x0.val", (int)(1000 * Kd));
}
void trigger34() //Load Calibration Offset
{
  THCNex.writeNum("CustomSetPoint.val", CalibrationOffset);
}
void trigger35() //Save Calibration Offset on end touch event
{
  int cali = THCNex.readNumber("CustomSetPoint.val");
  if (cali != 77777) {
    CalibrationOffset = cali;
    OpenSaved();
    Saved.putInt("Calibrate", (int)CalibrationOffset);
    OpenSaved(false);
  }
}
void report() //report plasma voltage and position
{
  THCNex.writeNum("PV.val", (int)Input);
  THCNex.writeNum("POS.val", (int)(pos / 2));
}
void format() //Set text color
{
  if (pos > 1 && pos < 1000)
  {
    THCNex.writeNum("POS.pco", 4065);
  }
  if (pos < -1 && pos > -1000)
  {
    THCNex.writeNum("POS.pco", 63488);
  }
}
