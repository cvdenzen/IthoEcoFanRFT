
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
// #define DISABLE_MYSENSORS

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
#define MY_RFM69_FREQUENCY   RFM69_868MHZ
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
unsigned long lastCommandDateTime=0;

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Start ArjenHiemstra IthoEcoFan

//#include <SPI.h>
#include "IthoCC1101.h"
#include "IthoPacket.h"

#define ITHO_IRQ_PIN 3 //cc1101 pin GD02, connected to pin 3 on Carl's Arduino Pro Mini D2(GPIO4) on NodeMCU

IthoCC1101 rf;
IthoPacket packet;

const uint8_t RFTid[] = {11, 22, 33}; // my ID

volatile bool ITHOhasPacket = false;
IthoCommand RFTcommand[3] = {IthoUnknown, IthoUnknown, IthoUnknown};
byte RFTRSSI[3] = {0, 0, 0};
byte RFTcommandpos = 0;
IthoCommand RFTlastCommand = IthoLow;
IthoCommand RFTstate = IthoUnknown;
IthoCommand savedRFTstate = IthoUnknown;
bool RFTidChk[3] = {false, false, false};
// End of ArjenHiemstra IthoEcoFan
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <DallasTemperature.h>
#include <OneWire.h>

#define COMPARE_TEMP 1 // Send temperature only if changed? 1 = Yes 0 = No

#define ONE_WIRE_BUS 8 // Pin where dallas sensor is connected
#define MAX_ATTACHED_DS18B20 16

#ifdef DISABLE_MYSENSORS
#include <EEPROM.h>
// MySensors uses some EEPROM, and defines the start
#define EEPROM_START (700)
#endif
// To make the index constant, use this initialiser
const DeviceAddress knownDeviceAddresses[] = {
  { 0x28, 0xB8, 0xB4, 0xF6, 0x0D, 0x00, 0x00, 0x80 },  // sensor on pcb
  { 0x28, 0xF8, 0xC6, 0x16, 0x3A, 0x19, 0x01, 0xF6 },  // waterproof, hot water sensor
  { 0x28, 0x38, 0xC0, 0x1F, 0x0E, 0x00, 0x00, 0x26 },  // cold water
  { 0x28, 0xB6, 0xC6, 0x1E, 0x0E, 0x00, 0x00, 0xCC },  // south-east (front) hot out heater
  { 0x28, 0x83, 0x91, 0x1F, 0x0E, 0x00, 0x00, 0x81 },  // se return
  { 0x28, 0x7F, 0xAB, 0xF6, 0x0D, 0x00, 0x00, 0x89 },  // nw (back) hot out heater
  { 0x28, 0x7F, 0xA7, 0xF6, 0x0D, 0x00, 0x00, 0xA8 }   // nw return
};
uint16_t firstFreeInRom = EEPROM_START + sizeof(knownDeviceAddresses);
uint8_t entriesInRom = 0;
#define EEPROM_TABLE_END (EEPROM_START + MAX_ATTACHED_DS18B20 * sizeof(DeviceAddress))

#define LED_PIN LED_BUILTIN
#define FADE_DELAY 10  // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim)

unsigned long SLEEP_TIME = 1000; // Sleep time between reads (in milliseconds)


OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
float lastTemperature[MAX_ATTACHED_DS18B20];
unsigned long lastTemperatureUpdateMillis[MAX_ATTACHED_DS18B20];
bool receivedConfig = false;
bool metric = true;
// Initialize temperature message
MyMessage msg(0,V_TEMP);
static int16_t currentLevel = 0;  // Current dim level..
#define MYSENSORS_DALLAS_PRESENTATION_OFFSET (20)
MyMessage fanMsg(0,V_VAR1); // custom, from controller to mysensor to fan
MyMessage fromCC1101(0,V_VAR1); // custom, from CC1101 to controller (openhab)
MyMessage idFromCC1101(0,V_VAR2); // custom, first 3 id fields (to detect interfering itho remotes)

// Optional method - for initialisations that needs to take place before MySensors transport has been setup (eg: SPI devices).
void before()
{
  wdt_disable();
  pinMode(LED_PIN,OUTPUT);
  // Startup up the OneWire library
#ifdef MY_DEBUG
  Serial.println(F("before(): between wdt_disable and sensors.begin"));
#endif
  sensors.begin();
  //Serial.println("before(): after sensors.begin");
}

void setup()
{
  setupArjen();
  DeviceAddress deviceAddress;
  #ifdef MY_DEBUG
  Serial.begin(38400);
  while (!Serial)
    ;
  #endif
  #ifdef MY_DEBUG
  Serial.print(F("==================== Start setup for mysensors node number "));
  Serial.println(MY_NODE_ID);
  #endif
  for (int i=0;i<MAX_ATTACHED_DS18B20;i++) {
    lastTemperatureUpdateMillis[i]=0;
    lastTemperature[i]=-150.0;
  }

  oneWire.reset_search();
  copyKnownDeviceAddressesToEEPROM();
  #ifdef MY_DEBUG
  Serial.print(F("setup: EEPROM_START,firstFreeInRom="));
  Serial.print(EEPROM_START);
  Serial.print(F(", "));
  Serial.println((uint32_t)firstFreeInRom);
  Serial.print(F("Setup, numSensors="));
  Serial.print(sensors.getDeviceCount());
  Serial.print(F(", MAX_ATTACHED_DS18B20="));
  Serial.println(MAX_ATTACHED_DS18B20);
  #endif
  uint8_t i = 0;
  while (oneWire.search(deviceAddress)) {
    #ifdef MY_DEBUG
    Serial.print(F("setup: found from search deviceAddress:"));
    printlnDeviceAddress(deviceAddress);
    #endif
    //printDallasTemperature(oneWire,deviceAddress);
    //for (uint8_t j=0;j<8;j++) deviceAddresses[i][j]=deviceAddress[j];
    //Serial.println(F("Copied from .. to:"));printDeviceAddress(deviceAddress);printDeviceAddress(deviceAddresses[i]);Serial.println(F("Finished copy"));
    // Lookup if known address
    bool found = false;
    for (uint8_t i1 = 0; i1 < MAX_ATTACHED_DS18B20 && i1 < MAX_ATTACHED_DS18B20; i1++) {
      #ifdef MY_DEBUG
      #ifdef DISABLE_MYSENSORS
      Serial.print(F("EEPROM_START+i1*sizeof(deviceAddress)="));
      Serial.println(EEPROM_START + i1 * sizeof(deviceAddress));
      #endif
      Serial.print(F("firstFreeInRom="));Serial.print(firstFreeInRom);
      Serial.print(F(", entriesInRom="));Serial.print(entriesInRom);
      Serial.print(F(", i1="));
      Serial.print(i1);      
      #endif
      DeviceAddress da;
      getRomEntry(i1,da);
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
    if (!found && firstFreeInRom < EEPROM_TABLE_END) {
      // New address
      #ifdef MY_DEBUG
      Serial.print(F("Found new device:"));
      printDeviceAddress(deviceAddress);
      Serial.print(F(", write to rom "));
      Serial.println(firstFreeInRom);
      #endif
      // write to rom
      addDeviceToRom(deviceAddress);
      #ifdef MY_DEBUG
      Serial.print(F("Device added, New firstFreeInRom: "));
      Serial.println(firstFreeInRom);
      #endif
    }
    i++;
  }  // while search for devices
  #ifdef MY_DEBUG
  Serial.print(F("End setup for node number "));Serial.println(MY_NODE_ID);
#endif
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  //Serial.print("Start presentation for node number ");Serial.print(MY_NODE_ID);
  sendSketchInfo("FanAttic_156", "1.1");

  // Fetch the number of attached temperature sensors
  // numSensors = sensors.getDeviceCount();

  uint8_t i;
  // was 7..10
  i=7;
  present(i++,S_TEMP); // The original temperature sensor
  present(i++,S_CUSTOM); // from controller to itho fan
  present(i++,S_CUSTOM); // from cc1101 receiver to controller (openhab)
  present(i++,S_CUSTOM); // id in packet from cc1101 receiver to controller
  // Present all sensors to controller, mySensors start with 1
  for (i=MYSENSORS_DALLAS_PRESENTATION_OFFSET; i<MAX_ATTACHED_DS18B20+MYSENSORS_DALLAS_PRESENTATION_OFFSET; i++) {
     present(i, S_TEMP);
     wait(200); // to avoid NACK problems?
  }
}

void receive(const MyMessage &message) {

#ifdef MY_DEBUG
  Serial.print( F("Message received ") );Serial.println(atoi( message.data ));
#endif
  if (message.getType() == V_VAR1) {

    //  Retrieve from the incoming message
    int receivedIthoCommand = atoi( message.data );
    noInterrupts();
    // Enqueue the command a few times, so it will be sent multiple times.
    // We do not want the send to be done in the receive function, that would block
    // the receive ?
    for (int i=0;i<1;i++) {
      cc1101commandQueue.enqueue(receivedIthoCommand);
    }
    interrupts();
  }
}
void loop()
{
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
  DeviceAddress deviceAddress;
  // Read temperatures and send them to controller
  for (int i=0; i<MAX_ATTACHED_DS18B20; i++) {
    getRomEntry(i,deviceAddress);
    // Fetch and round temperature to one decimal
    if (deviceAddress[0]==0x28) {
      float temperature = 
        static_cast<float>(static_cast<int>((getControllerConfig()
                     .isMetric?sensors.getTempC(deviceAddress):sensors.getTempF(deviceAddress)) * 10.)) / 10.;

      // Only send data if temperature has changed and no error
      //if (temperature != -127.00 && temperature != 85.00
      if (temperature > -126.00 && (temperature < 84.95 or temperature > 85.05)
      #if COMPARE_TEMP == 1
          && lastTemperature[i] != temperature
      #endif
        ) {
        // Do not send too often
        float tempDiff=temperature-lastTemperature[i]; // to suit the abs macro
        if ( ( (millis()-lastTemperatureUpdateMillis[i])>60000)
            ||(abs(tempDiff)>0.5)) {
          // Send in the new temperature with 1 decimal
          send(msg.setSensor(i+MYSENSORS_DALLAS_PRESENTATION_OFFSET).set(temperature,1),false);
          wait(1000);
          // Save new temperatures for next compare
          lastTemperature[i]=temperature;
          lastTemperatureUpdateMillis[i]=millis();
        }
      }
    }
  } // for int i=0..

  #ifndef DISABLE_MYSENSORS
  wait(SLEEP_TIME);
  #else
  delay(SLEEP_TIME);
  #endif
}
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Start ArjenHiemstra Itho

void setupArjen(void) {
  //Serial.begin(115200);
  pinMode(ITHO_IRQ_PIN, INPUT);
  pinMode(CC1101_SS,OUTPUT); // ??
  delay(500);
  //Serial.println("setupArjen begin");
  rf.setDeviceID(13, 123, 42); //DeviceID used to send commands, can also be changed on the fly for multi itho control
  //Serial.println("setupArjen rf.setDeviceID() done");
  rf.init();
  //Serial.println("setupArjen rf.init() done");
  //sendRegister();
  //Serial.println("join command sent");
  pinMode(ITHO_IRQ_PIN, INPUT);
  pinMode(CC1101_SS,OUTPUT); // ??
  attachInterrupt(digitalPinToInterrupt(ITHO_IRQ_PIN), ITHOcheck, FALLING);

  //rf.sendCommand(IthoJoin);
  //rf.sendCommand(IthoLow);
  delay(2000);

  //rf.sendCommand(IthoTimer1);
  //delay(1000);
}

void loopArjen(void) {
  // do whatever you want, check (and reset) the ITHOhasPacket flag whenever you like
  if (ITHOhasPacket) {
    ITHOhasPacket = false;
    if (rf.checkForNewPacket()) {
      IthoCommand cmd = rf.getLastCommand();
      if (++RFTcommandpos > 2) RFTcommandpos = 0;  // store information in next entry of ringbuffers
      RFTcommand[RFTcommandpos] = cmd;
      RFTRSSI[RFTcommandpos]    = rf.ReadRSSI();
      bool chk = rf.checkID(RFTid);
      RFTidChk[RFTcommandpos]   = chk;
      if ((cmd != IthoUnknown)) {  // only act on good cmd and correct id.
        showPacket();
      }
    }
  }
  int receivedIthoCommand;
  noInterrupts();
  if (!cc1101commandQueue.isEmpty() && ((millis()-lastCommandDateTime)>1000)) {
    receivedIthoCommand=cc1101commandQueue.dequeue();
    interrupts();
    pinMode(ITHO_IRQ_PIN, INPUT);
    pinMode(CC1101_SS,OUTPUT); // ??
    attachInterrupt(digitalPinToInterrupt(ITHO_IRQ_PIN), ITHOcheck, FALLING);
    // Send x times
    #ifdef MY_DEBUG
    Serial.print(F("Sending queued command to cc1101: "));Serial.println(receivedIthoCommand);
    #endif
    for (int i=0;i<1;i++) {
      rf.sendCommand(receivedIthoCommand);
    }
    lastCommandDateTime=millis();
  } else {
      interrupts();
  }
}

/* ICACHE_RAM_ATTR ?? */ void ITHOcheck() {
  ITHOhasPacket = true;
}

void showPacket() {
  uint8_t goodpos = findRFTlastCommand();
  if (goodpos != -1)  RFTlastCommand = RFTcommand[goodpos];
  else                RFTlastCommand = IthoUnknown;
  //show data
  /* save memory
  Serial.print(F("RFT Current Pos: "));
  Serial.print(RFTcommandpos);
  Serial.print(F(", Good Pos: "));
  Serial.println(goodpos);
  Serial.print(F("Stored 3 commands: "));
  Serial.print(RFTcommand[0]);
  Serial.print(F(" "));
  Serial.print(RFTcommand[1]);
  Serial.print(F(" "));
  Serial.print(RFTcommand[2]);
  Serial.print(F(" / Stored 3 RSSI's:     "));
  Serial.print(RFTRSSI[0]);
  Serial.print(F(" "));
  Serial.print(RFTRSSI[1]);
  Serial.print(F(" "));
  Serial.print(RFTRSSI[2]);
  Serial.print(F(" / Stored 3 ID checks: "));
  Serial.print(RFTidChk[0]);
  Serial.print(F(" "));
  Serial.print(RFTidChk[1]);
  Serial.print(F(" "));
  Serial.print(RFTidChk[2]);
  Serial.print(F(" / Last ID: "));
  Serial.print(rf.getLastIDstr(false));
  

  Serial.print(F(" / Command = "));
  */
  //show command
#ifdef MY_DEBUG
  Serial.print(F("Received command on CC1101 receiver: "));
  switch (RFTlastCommand) {
    case IthoUnknown:
      Serial.print(F("unknown\n"));
      break;
    case IthoLow:
      Serial.print(F("low\n"));
      break;
    case IthoMedium:
      Serial.print(F("medium\n"));
      break;
    case IthoHigh:
      Serial.print(F("high\n"));
      break;
    case IthoFull:
      Serial.print(F("full\n"));
      break;
    case IthoTimer1:
      Serial.print(F("timer1\n"));
      break;
    case IthoTimer2:
      Serial.print(F("timer2\n"));
      break;
    case IthoTimer3:
      Serial.print(F("timer3\n"));
      break;
    case IthoJoin:
      Serial.print(F("join\n"));
      break;
    case IthoLeave:
      Serial.print(F("leave\n"));
      break;
  }
#endif
}

uint8_t findRFTlastCommand() {
  if (RFTcommand[RFTcommandpos] != IthoUnknown)               return RFTcommandpos;
  if ((RFTcommandpos == 0) && (RFTcommand[2] != IthoUnknown)) return 2;
  if ((RFTcommandpos == 0) && (RFTcommand[1] != IthoUnknown)) return 1;
  if ((RFTcommandpos == 1) && (RFTcommand[0] != IthoUnknown)) return 0;
  if ((RFTcommandpos == 1) && (RFTcommand[2] != IthoUnknown)) return 2;
  if ((RFTcommandpos == 2) && (RFTcommand[1] != IthoUnknown)) return 1;
  if ((RFTcommandpos == 2) && (RFTcommand[0] != IthoUnknown)) return 0;
  return -1;
}

void sendRegister() {
  Serial.println(F("sending join..."));
  rf.sendCommand(IthoJoin);
  Serial.println(F("sending join done."));
}

void sendStandbySpeed() {
  Serial.println(F("sending standby..."));
  rf.sendCommand(IthoStandby);
  Serial.println(F("sending standby done."));
}

void sendLowSpeed() {
  Serial.println(F("sending low..."));
  rf.sendCommand(IthoLow);
  Serial.println(F("sending low done."));
}

void sendMediumSpeed() {
  Serial.println(F("sending medium..."));
  rf.sendCommand(IthoMedium);
  Serial.println(F("sending medium done."));
}

void sendHighSpeed() {
  Serial.println(F("sending high..."));
  rf.sendCommand(IthoHigh);
  Serial.println(F("sending high done."));
}

void sendFullSpeed() {
  Serial.println(F("sending FullSpeed..."));
  rf.sendCommand(IthoFull);
  Serial.println(F("sending FullSpeed done."));
}

void sendTimer() {
  Serial.println(F("sending timer..."));
  rf.sendCommand(IthoTimer1);
  Serial.println(F("sending timer done."));
}

  // End of ArjenHiemstra's IthoEcoFan
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void copyKnownDeviceAddressesToEEPROM() {
  // put, only updated data, into rom from first free address from mysensors
  #ifdef DISABLE_MYSENSORS
  EEPROM.put(EEPROM_START, knownDeviceAddresses);
  firstFreeInRom=EEPROM_START+sizeof(knownDeviceAddresses);
  // Clear other entries
  for (uint16_t i=firstFreeInRom;i<EEPROM_TABLE_END;i++) EEPROM.update(i,0);
  #else
  // use mysensors EEPROM management
  uint8_t i=0;
  uint8_t *p=&knownDeviceAddresses[0][0];
  for (;i<sizeof(knownDeviceAddresses);i++) {
    saveState(i,*p++);
    #ifdef MY_DEBUG
    //Serial.print(F("copyKnownDeviceAdresses i="));Serial.print(i);Serial.print(F(", p="));Serial.print((uint32_t)p);
    //Serial.print(F(", loadState(i)="));Serial.println(loadState(i),HEX);
    #endif
  }
  firstFreeInRom=i;
  // Clear other entries
  for (;i<MAX_ATTACHED_DS18B20 * sizeof(DeviceAddress) && i<255;i++) {
    saveState(i,0);
  }
  #endif
  entriesInRom = sizeof(knownDeviceAddresses) / sizeof(DeviceAddress);
  #ifdef MY_DEBUG
  printRomContents();
  #endif
}
void addDeviceToRom(DeviceAddress da) {
  for (uint8_t i=0;i<sizeof(DeviceAddress);i++) {
    #ifdef DISABLE_MYSENSORS
    EEPROM.put(firstFreeInRom++, da[i]);
    #else
    // mysensors has own functions for eeprom management
    saveState(firstFreeInRom++,da[i]);
    #endif
  }
  #ifdef MY_DEBUG
  Serial.print(F("addDeviceToRom2: sizeof(*da)="));Serial.println(sizeof(*da));
  #endif
  entriesInRom++;
  printRomContents();
  // Mark last entry: first byte 0, unless the table is full
  if (firstFreeInRom < EEPROM_TABLE_END) {
    #ifdef DISABLE_MYSENSORS
    EEPROM.put(firstFreeInRom, (uint8_t)0);
    #else
    saveState(firstFreeInRom,0);
    #endif
    #ifdef MY_DEBUG
    Serial.print(F("write table end at firstFreeInRom: "));
    Serial.println(firstFreeInRom);
    #endif
  }
  #ifdef MY_DEBUG
  Serial.print(F("addDeviceToRom: firstFreeInRom="));
  Serial.print(firstFreeInRom);
  Serial.print(F(", entriesInRom="));
  Serial.println(entriesInRom);
  #endif
}
void printRomContents() {
  #ifndef DISABLE_MYSENSORS;
  uint8_t rp=0; // rom pointer
  #endif
  for (uint8_t i = 0; i < MAX_ATTACHED_DS18B20; i++) {
    // Print the address to EEPROM
    DeviceAddress da;
    #ifdef DISABLE_MYSENSORS
    EEPROM.get(EEPROM_START + i * sizeof(DeviceAddress), da);
    #else
    for (uint8_t j=0;j<sizeof(DeviceAddress);j++) {
      uint8_t x=loadState(rp++);
      da[j]=x;
    }
    #endif
    Serial.print(F("printRomContents, i="));Serial.print(i);Serial.print(F(""));printlnDeviceAddress(da);
  }
}
void getRomEntry(uint8_t index, uint8_t *da_par) {
  //DeviceAddress da;
  #ifdef DISABLE_MYSENSORS
  EEPROM.get(EEPROM_START + index * sizeof(DeviceAddress), da_par);
  #else
  uint8_t j=0;
  for (uint8_t i=index*sizeof(DeviceAddress);j<sizeof(DeviceAddress);i++,j++) {
    da_par[j]=loadState(i);
  }
  #endif
  //da_par = da; // ?????????????
  #ifdef MY_DEBUG
  Serial.print(F("getRomEntry, index, da="));
  Serial.print(index);Serial.print(F(", "));
  printlnDeviceAddress(da_par);
  #endif
}
#ifdef MY_DEBUG
void printDeviceAddress(DeviceAddress deviceAddress) {
  Serial.print(F(" da="));
  for (uint8_t i = 0; i < 8; i++) {
    //Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print("-");
  }
}
void printlnDeviceAddress(DeviceAddress deviceAddress) {
  printDeviceAddress(deviceAddress);
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
  #ifdef MY_DEBUG
  Serial.print(F("P="));
  Serial.print(present, HEX);
  Serial.print(F(" "));
  #endif
  uint8_t data[8];
  for (uint8_t i = 0; i < 9; i++) {  // we need 9 bytes
    data[i] = oneWire.read();
    #ifdef MY_DEBUG
    Serial.print(data[i], HEX);
    Serial.print(F(" "));
    #endif
  }
  #ifdef MY_DEBUG
  Serial.print(F(" CRC="));
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
  #endif
}
