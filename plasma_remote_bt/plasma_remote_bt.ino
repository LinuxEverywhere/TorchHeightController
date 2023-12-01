//#include <BTAddress.h>
//#include <BTScan.h>
#include <BluetoothSerial.h>
//#include <BTAdvertisedDevice.h>

#define PLASMA_INPUT_PIN 36

unsigned long m, lastreadTime = 0;
unsigned long t = 500;
uint8_t PlasmaV_last,PlasmaV = 0;

//#define USE_NAME // Comment this to use MAC address instead of a slaveName
const char *pin = "43f4"; // Change this to reflect the pin expected by the real slave BT device

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

#ifdef USE_NAME
  String slaveName = "PlasmaTHC"; // Change this to reflect the real name of your slave BT device
#else
  String MACadd = "AA:BB:CC:11:22:33"; // This only for printing
  uint8_t address[6]  = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33}; // Change this to reflect real MAC address of your slave BT device
#endif

String myName = "THCRemote";
bool connected;

void BT_connect() {
  
  // connect(address) is fast (up to 10 secs max), connect(slaveName) is slow (up to 30 secs max) as it needs
  // to resolve slaveName to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices Bluetooth address and device names
  #ifdef USE_NAME
    connected = SerialBT.connect(slaveName);
    Serial.printf("Connecting to slave BT device named \"%s\"\n", slaveName.c_str());
  #else
    connected = SerialBT.connect(address);
    Serial.print("Connecting to slave BT device with MAC "); Serial.println(MACadd);
  #endif

  if(connected) {
    Serial.println("Connected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
    }
  }
  // Disconnect() may take up to 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Successfully!");
  }
  // This would reconnect to the slaveName(will use address, if resolved) or address used with connect(slaveName/address).
  SerialBT.connect();
  if(connected) {
    Serial.println("Reconnected Successfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to reconnect. Make sure remote device is available and in range, then restart app.");
    }
  }
}

void setup() {
  
  Serial.begin(115200);
  SerialBT.begin(myName, true);
  Serial.printf("The device \"%s\" started in master mode, make sure slave BT device is on!\n", myName.c_str());
  #ifndef USE_NAME
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
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
