
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
#define MY_BAUD_RATE 38400


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

#include <MySensors.h>

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
#define MAX_ATTACHED_DS18B20 1

#define LED_PIN LED_BUILTIN
#define FADE_DELAY 10  // Delay in ms for each percentage fade up/down (10ms = 1s full-range dim)

unsigned long SLEEP_TIME = 3000; // Sleep time between reads (in milliseconds)


OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass the oneWire reference to Dallas Temperature.
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
bool receivedConfig = false;
bool metric = true;
// Initialize temperature message
MyMessage msg(0,V_TEMP);
static int16_t currentLevel = 0;  // Current dim level..

MyMessage fanMsg(0,V_VAR1); // custom, from controller to mysensor to fan
MyMessage fromCC1101(0,V_VAR1); // custom, from CC1101 to controller (openhab)
MyMessage idFromCC1101(0,V_VAR2); // custom, first 3 id fields (to detect interfering itho remotes)


void before()
{
  wdt_disable();
  pinMode(LED_PIN,OUTPUT);
  // Startup up the OneWire library
  Serial.println("before(): between wdt_disable and sensors.begin");
  sensors.begin();
  //Serial.println("before(): after sensors.begin");
  

}

void setup()
{
  setupArjen();
  // Pull the gateway's current dim level - restore light level upon node power-up
  Serial.print("End setup for node number ");Serial.println(MY_NODE_ID);

}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  //Serial.print("Start presentation for node number ");Serial.print(MY_NODE_ID);
  sendSketchInfo("FanAttic_156", "1.1");

  // Fetch the number of attached temperature sensors
  numSensors=1; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!numSensors = sensors.getDeviceCount();

  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     present(i+1, S_TEMP);
  }

  present(7,S_DIMMER);
  present(8,S_CUSTOM); // from controller to itho fan
  present(9,S_CUSTOM); // from cc1101 receiver to controller (openhab)
  present(10,S_CUSTOM); // id in packet from cc1101 receiver to controller
}

void receive(const MyMessage &message) {

  Serial.print( "Message received " );
  if (message.getType() == V_VAR1) {

    //  Retrieve the power or dim level from the incoming request message
    int receivedIthoCommand = atoi( message.data );
    Serial.print("Received command ");Serial.println(receivedIthoCommand);

  pinMode(ITHO_IRQ_PIN, INPUT);
  pinMode(CC1101_SS,OUTPUT); // ??
  attachInterrupt(digitalPinToInterrupt(ITHO_IRQ_PIN), ITHOcheck, FALLING);

  delay(1000);
  switch(receivedIthoCommand) {
    
    case 34: // low
      rf.sendCommand(IthoLow);
      break;
    
    case 35: // low
      rf.sendCommand(IthoLow);
      break;
    case 41: // timer1
      rf.sendCommand(IthoTimer1);
      break;
    default: // enum 0..12
      rf.sendCommand(receivedIthoCommand);
      break;
  }
  delay(1000);
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
  wait(conversionTime);

  // Read temperatures and send them to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {

    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((getControllerConfig().isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;

    // Only send data if temperature has changed and no error
    #if COMPARE_TEMP == 1
    if (lastTemperature[i] != temperature && temperature != -127.00 && temperature != 85.00) {
    #else
    if (temperature != -127.00 && temperature != 85.00) {
    #endif
       // Send in the new temperature with 1 decimal
      send(msg.setSensor(i).set(temperature,1),false);
      // Save new temperatures for next compare
      lastTemperature[i]=temperature;
    }
  }

  loopArjen();
  
  
  // sleep with time is difficult
  delay(SLEEP_TIME);
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

  rf.sendCommand(IthoTimer1);
  delay(1000);
}

void loopArjen(void) {
  // do whatever you want, check (and reset) the ITHOhasPacket flag whenever you like
  if (ITHOhasPacket) {
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
}

/* ICACHE_RAM_ATTR ?? */ void ITHOcheck() {
  ITHOhasPacket = true;
}

void showPacket() {
  ITHOhasPacket = false;
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
  switch (RFTlastCommand) {
    case IthoUnknown:
      Serial.print("unknown\n");
      break;
    case IthoLow:
      Serial.print("low\n");
      break;
    case IthoMedium:
      Serial.print("medium\n");
      break;
    case IthoHigh:
      Serial.print("high\n");
      break;
    case IthoFull:
      Serial.print("full\n");
      break;
    case IthoTimer1:
      Serial.print("timer1\n");
      break;
    case IthoTimer2:
      Serial.print("timer2\n");
      break;
    case IthoTimer3:
      Serial.print("timer3\n");
      break;
    case IthoJoin:
      Serial.print("join\n");
      break;
    case IthoLeave:
      Serial.print("leave\n");
      break;
  }
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
  Serial.println("sending join...");
  rf.sendCommand(IthoJoin);
  Serial.println("sending join done.");
}

void sendStandbySpeed() {
  Serial.println("sending standby...");
  rf.sendCommand(IthoStandby);
  Serial.println("sending standby done.");
}

void sendLowSpeed() {
  Serial.println("sending low...");
  rf.sendCommand(IthoLow);
  Serial.println("sending low done.");
}

void sendMediumSpeed() {
  Serial.println("sending medium...");
  rf.sendCommand(IthoMedium);
  Serial.println("sending medium done.");
}

void sendHighSpeed() {
  Serial.println("sending high...");
  rf.sendCommand(IthoHigh);
  Serial.println("sending high done.");
}

void sendFullSpeed() {
  Serial.println("sending FullSpeed...");
  rf.sendCommand(IthoFull);
  Serial.println("sending FullSpeed done.");
}

void sendTimer() {
  Serial.println("sending timer...");
  rf.sendCommand(IthoTimer1);
  Serial.println("sending timer done.");
}

  // End of ArjenHiemstra's IthoEcoFan
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
