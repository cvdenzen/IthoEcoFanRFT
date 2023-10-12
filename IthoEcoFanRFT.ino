
/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 *
 * Example sketch showing how to send in DS1820B OneWire temperature readings back to the controller
 * http://www.mysensors.org/build/temp
 */


// Enable debug prints to serial monitor
#define MY_DEBUG
//#undef MY_DEBUG
#define MY_BAUD_RATE 38400
// Disable mysensors (to test only Dallas sensors)
#define DISABLE_MYSENSORS

// Enable and select radio type attached
//#define MY_RADIO_RF24
#define MY_RADIO_RFM69
// If using mysensors version<2.3.0 and want to use NEW DRIVER (OH SO BAD NAMING):
#define MY_RFM69_NEW_DRIVER
// if you use MySensors 2.0 use this style
//#define MY_RFM69_FREQUENCY   RF69_433MHZ
//#define MY_RFM69_FREQUENCY   RF69_868MHZ
//#define MY_RFM69_FREQUENCY   RF69_915MHZ

// Do not use default 100 to avoid conflicts with neighbors that won't change this id
#define MY_RFM69_NETWORKID 197

//#define MY_RFM69_FREQUENCY   RFM69_433MHZ
#define MY_RFM69_FREQUENCY RFM69_868MHZ
//#define DEFAULT_RFM69_CS_PIN     (PB2)                      //!< DEFAULT_RFM69_CS_PIN
// default #define MY_RF69_IRQ_PIN PD2


//#define MY_RADIO_NRF24

// Comment it out for Auto Node ID #, original was 0x90=144
#define MY_NODE_ID 156

#ifndef DISABLE_MYSENSORS
#include <MySensors.h>
#else
#include <avr/wdt.h>
#endif
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include <ArduinoQueue.h>
ArduinoQueue<int> cc1101commandQueue(6);
unsigned long lastCommandDateTime = 0;


#include <DallasTemperature.h>
#include <OneWire.h>

#define COMPARE_TEMP 1  // Send temperature only if changed? 1 = Yes 0 = No

#define ONE_WIRE_BUS 8  // Pin where dallas sensor is connected
#define MAX_ATTACHED_DS18B20 12

#include <EEPROM.h>
// MySensors uses some EEPROM, and defines the start
#ifndef EEPROM_LOCAL_CONFIG_ADDRESS
#define EEPROM_START (700)
#else
#define EEPROM_START (EEPROM_LOCAL_CONFIG_ADDRESS)
#endif
// To make the index constant, use this initialiser
const DeviceAddress knownDeviceAddresses[] = {
  { 0x28, 0xF8, 0xC6, 0x16, 0x3A, 0x19, 0x01, 0xF6 },  // waterproof, hot water sensor
  { 0x28, 0x38, 0xC0, 0x1F, 0x0E, 0x00, 0x00, 0x26 },  // cold water
  { 0x28, 0xB6, 0xC6, 0x1E, 0x0E, 0x00, 0x00, 0xCC },  // south-east (front) hot out heater
  { 0x28, 0x83, 0x91, 0x1F, 0x0E, 0x00, 0x00, 0x81 },  // se return
  { 0x28, 0x7F, 0xAB, 0xF6, 0x0D, 0x00, 0x00, 0x89 },  // nw (back) hot out heater
  { 0x28, 0x7F, 0xA7, 0xF6, 0x0D, 0x00, 0x00, 0xA8 }   // nw return
};
DeviceAddress *firstFreeInRom = EEPROM_START + sizeof(knownDeviceAddresses);
uint8_t entriesInRom = 0;

#define LED_PIN LED_BUILTIN
#define FADE_DELAY 10  // Delay in ms for each percentage fade up/down (10ms = 1s full, 0xrange dim)

unsigned long SLEEP_TIME = 200;  // Sleep time between reads (in milliseconds)


OneWire oneWire(ONE_WIRE_BUS);                      // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire, ONE_WIRE_BUS);  // Pass the oneWire reference to Dallas Temperature.
DeviceAddress *deviceAddresses = EEPROM_START;
float lastTemperature[MAX_ATTACHED_DS18B20];
unsigned long lastTemperatureUpdateMillis[MAX_ATTACHED_DS18B20];
int numSensors = 0;
bool receivedConfig = false;
bool metric = true;
static int16_t currentLevel = 0;  // Current dim level..


void before() {
  // Disable watchdog, we are not running on battery, we are listening to commands from gateway
  wdt_disable();
  // Serial impossible in before()????
  //Serial.begin(38400);
  //while (!Serial);
  pinMode(LED_PIN, OUTPUT);
  // Startup up the OneWire library
#ifdef MY_DEBUG
  Serial.println(F("before(): between wdt_disable and sensors.begin"));
#endif
  //Serial.println("before(): after sensors.begin");

#ifdef MY_DEBUG
  Serial.println(F("End before()"));
#endif
}

void setup() {
  Serial.begin(38400);
  while (!Serial)
    ;
#ifdef MY_DEBUG
  Serial.print(F("==================== Start setup for mysensors node number "));
  Serial.println(MY_NODE_ID);
#endif
  for (int i = 0; i < MAX_ATTACHED_DS18B20; i++) {
    lastTemperatureUpdateMillis[i] = 0;
    lastTemperature[i] = 0.0;
  }

  sensors.begin();
  DeviceAddress deviceAddress;
  oneWire.reset_search();
  copyKnownDeviceAddressesToEEPROM();
#ifdef MY_DEBUG
  Serial.print(F("setup: EEPROM_START,firstFreeInRom="));
  Serial.print(EEPROM_START);
  Serial.print(F(", "));
  Serial.println((uint32_t)firstFreeInRom);
#endif
  uint8_t i = 0;
  while (oneWire.search(deviceAddress)) {
#ifdef MY_DEBUG
    Serial.print(F("setup: found from search deviceAddress:"));
    printDeviceAddress(deviceAddress);
    Serial.print(F("Setup, numSensors="));
    Serial.print(sensors.getDeviceCount());
    Serial.print(F(", MAX_ATTACHED_DS18B20="));
    Serial.println(MAX_ATTACHED_DS18B20);
#endif
    //printDallasTemperature(oneWire,deviceAddress);
    //for (uint8_t j=0;j<8;j++) deviceAddresses[i][j]=deviceAddress[j];
    //Serial.println(F("Copied from .. to:"));printDeviceAddress(deviceAddress);printDeviceAddress(deviceAddresses[i]);Serial.println(F("Finished copy"));
    // Lookup if known address
    bool found = false;
    for (uint8_t i1 = 0; i1 < MAX_ATTACHED_DS18B20 && i1 < entriesInRom; i1++) {
#ifdef MY_DEBUG
      Serial.print(F("EEPROM_START+i1*sizeof(deviceAddress)="));
      Serial.print(EEPROM_START + i1 * sizeof(deviceAddress));
      Serial.print(F(", i1="));
      Serial.print(i1);
      Serial.print(F(", sizeof(deviceAddress)="));
      Serial.println(sizeof(deviceAddress));
#endif
      DeviceAddress da;
      EEPROM.get(EEPROM_START + i1 * sizeof(deviceAddress), da);
      if (memcmp(deviceAddress, da, sizeof(deviceAddress)) == 0) {
// found a match
#ifdef MY_DEBUG
        Serial.print(F("Found known device:"));
        printDeviceAddress(deviceAddress);
        Serial.print(F(" in rom at index "));
        Serial.println(i1);
#endif
        found = true;
        break;
      }
    }
    if (!found) {
// New address
#ifdef MY_DEBUG
      Serial.print(F("Found new device:"));
      printDeviceAddress(deviceAddress);
      Serial.print(F(", write to rom "));
      Serial.println((uint32_t)firstFreeInRom);
#endif
      // write to rom
      EEPROM.put((uint32_t)firstFreeInRom, deviceAddress);
      firstFreeInRom++;
#ifdef MY_DEBUG
      Serial.print(F("New firstFreeInRom: "));
      Serial.println((uint32_t)firstFreeInRom);
#endif
    }
    i++;
  }
  sensors.requestTemperatures();
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  Serial.print(F(", conversionTime="));
  Serial.println(conversionTime);
  for (i = 0; i < entriesInRom; i++) {
    DeviceAddress da;
    getRomEntry(i, &da);
    // Fetch temperatures from Dallas sensors
    delay(conversionTime);
    Serial.print(F("Setup, getTEmpC for address "));
    printDeviceAddress(da);
    float tempCelcius = sensors.getTempC(da);
    Serial.print(F("Setup, Temp (Celcius)="));
    Serial.println(tempCelcius);
  }
#ifdef MY_DEBUG
  Serial.print(F("End of setup, numSensors="));
  Serial.print(sensors.getDeviceCount());
  Serial.print(F(", MAX_ATTACHED_DS18B20="));
  Serial.println(MAX_ATTACHED_DS18B20);
#endif
  delay(1000000);
}

void loop() {
#ifdef MY_DEBUG
  Serial.print(F("Start loop, numSensors="));
  Serial.print(sensors.getDeviceCount());
  Serial.print(F(", MAX_ATTACHED_DS18B20="));
  Serial.println(MAX_ATTACHED_DS18B20);
#endif
  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
// sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
//sleep(conversionTime);
#ifndef DISABLE_MYSENSORS
  wait(conversionTime);
#else
  delay(conversionTime);
#endif

  // Read temperatures and send them to controller
  for (int i = 0; i < numSensors && i < MAX_ATTACHED_DS18B20; i++) {

// Fetch and round temperature to one decimal
#ifndef DISABLE_MYSENSORS
    float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric ? sensors.getTempCByIndex(i) : sensors.getTempFByIndex(i)) * 10.)) / 10.;
#else
    float temperature = static_cast<float>(static_cast<int>((sensors.getTempCByIndex(i)) * 10.)) / 10.;
#endif

    // Only send data if temperature has changed and no error
    if (temperature != -127.00 && temperature != 85.00
#if COMPARE_TEMP == 1
        && lastTemperature[i] != temperature
#endif
    ) {
      // Do not send too often
      float tempDiff = temperature - lastTemperature[i];  // to suit the abs macro
      if (((millis() - lastTemperatureUpdateMillis[i]) > 60000)
          || (abs(tempDiff) > 2.0)) {
        // Send in the new temperature with 1 decimal
#ifndef DISABLE_MYSENSORS
        send(msg.setSensor(i).set(temperature, 1), false);
#endif
        // Save new temperatures for next compare
        lastTemperature[i] = temperature;
        lastTemperatureUpdateMillis[i] = millis();
      }
    }
  }  // for int i=0..

#ifndef DISABLE_MYSENSORS
  wait(SLEEP_TIME);
#else
  delay(SLEEP_TIME);
#endif
}

void copyKnownDeviceAddressesToEEPROM() {
  // put, only updated data, into rom from first free address from mysensors
  EEPROM.put(EEPROM_START, knownDeviceAddresses);
  //firstFreeInRom=EEPROM_START+sizeof(knownDeviceAddresses);
  entriesInRom = sizeof(knownDeviceAddresses) / sizeof(DeviceAddress);
#ifdef MY_DEBUG
  printRomContents();
#endif
}
void addDeviceToRom(DeviceAddress da) {
  EEPROM.put((uint16_t)firstFreeInRom, da);
  entriesInRom++;
#ifdef MY_DEBUG
  Serial.print(F("addDeviceToRom: firstFreeInRom="));
  Serial.print((uint16_t)firstFreeInRom);
  Serial.print(F(", entriesInRom="));
  Serial.println(entriesInRom);
#endif
}
void printRomContents() {
  for (uint8_t i = 0; i < MAX_ATTACHED_DS18B20; i++) {
    Serial.print("printRomContents, i=");
    Serial.println(i);
    // Print the address to EEPROM
    DeviceAddress da;
    EEPROM.get(EEPROM_START + i * sizeof(DeviceAddress), da);
    printDeviceAddress(da);
  }
}
void getRomEntry(uint8_t index, DeviceAddress *da) {
  //DeviceAddress da;
  EEPROM.get(EEPROM_START + index * sizeof(DeviceAddress), da);
  Serial.print(F("getRomEntry, da="));printDeviceAddress((uint8_t*)da);Serial.print(F(", index="));Serial.println(index);
}
#ifdef MY_DEBUG
void printDeviceAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    //Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print("-");
  }
  Serial.println("");
}
#endif
void printDallasTemperature(OneWire oneWire, DeviceAddress addr) {
  // The DallasTemperature library can do all this work for you!

  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0x44, 1);  // start conversion, with parasite power on at the end

  delay(1000);  // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  boolean present = oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0xBE);  // Read Scratchpad

  Serial.print("P=");
  Serial.print(present, HEX);
  Serial.print(" ");
  uint8_t data[8];
  for (uint8_t i = 0; i < 9; i++) {  // we need 9 bytes
    data[i] = oneWire.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
}
// End of ArjenHiemstra's IthoEcoFan
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
