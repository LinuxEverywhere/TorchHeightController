/*
    Plasma remote reader

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
  THC_Controller adjusts the torch voltage via Zaxis moves, using voltage data from THC_Remote
  This adds a Bluetooth module to the plasma cutter
  For Plasma cutters without a 50:1 output.

  Hardware:
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  1 x Esp32 dev board
  200:3.3 stepdown voltage divider circuit
  Power:LM317 5.0v circuit tapped into plasma 24V rail

  3rd Party Software:
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  BluetoothSerial
  By Evandro Copercini
  Version Used: 2.0.0
  Preferences
  ByHristo Gochkov
  Version Used: 2.0.0
  From: Additional Boards Manager URL: https://dl.espressif.com/dl/package_esp32_index.json
*/
#include <BluetoothSerial.h>
#define PLASMA_INPUT_PIN 4

unsigned long m, lastreadTime = 0;
unsigned long t = 200;
uint8_t PlasmaV_last,PlasmaV = 0;
bool debug = true;

//#define USE_NAME // Comment this to use MAC address instead of a slaveName
const char *pin = "43f4"; // Change this to reflect the pin expected by the real slave BT device

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
  String slaveName = "THC_Controller"; // Change this to reflect the real name of your slave BT device
#else
  String MACadd = "AA:BB:CC:11:22:33"; // This only for printing
  uint8_t address[6]  = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33}; // Change this to reflect real MAC address of your slave BT device
#endif

String myName = "THC_Remote";
bool connected;

void BT_connect() {

  // connect(address) is fast (up to 10 secs max), connect(slaveName) is slow (up to 30 secs max) as it needs
  // to resolve slaveName to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices Bluetooth address and device names
  #ifdef USE_NAME
    connected = SerialBT.connect(slaveName);
    if (debug) { Serial.printf("Connecting to slave BT device named \"%s\"\n", slaveName.c_str()); }
  #else
    connected = SerialBT.connect(address);
    if (debug) { Serial.print("Connecting to slave BT device with MAC "); Serial.println(MACadd); }
  #endif

  if(connected) {
    if (debug) { Serial.println("Connected Successfully!"); }
  } else {
    while(!SerialBT.connected(10000)) {
      if (debug) { Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");}
    }
  }
  // Disconnect() may take up to 10 secs max
  if (SerialBT.disconnect()) {
    if (debug) { Serial.println("Disconnected Successfully!");}
  }
  // This would reconnect to the slaveName(will use address, if resolved) or address used with connect(slaveName/address).
  SerialBT.connect();
  if(connected) {
    if (debug) { Serial.println("Reconnected Successfully!");}
  } else {
    while(!SerialBT.connected(10000)) {
      if (debug) { Serial.println("Failed to reconnect. Make sure remote device is available and in range, then restart app.");}
    }
  }
}

void setup() {

  if (debug) { Serial.begin(115200); }
  SerialBT.begin(myName, true);
  if (debug) { Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str()); }
  #ifndef USE_NAME
    SerialBT.setPin(pin);
    if (debug) { Serial.println("Using PIN"); }
  #endif
  BT_connect();
  analogReadResolution(9);
}

void loop() {

  m = millis();
  if (!connected) {
    BT_connect();
  }
  if (m-t>lastreadTime) {
    lastreadTime = m;
    PlasmaV = map(analogRead(PLASMA_INPUT_PIN), 0, 511, 0, 200000);
  }
  if (SerialBT.available()) {
    if (SerialBT.readString().indexOf("get") >= 0){
      SerialBT.write(PlasmaV);
    }
  }
}
