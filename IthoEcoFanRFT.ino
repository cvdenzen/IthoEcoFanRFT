
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
static int16_t currentLevel = 0;  // Current dim level...
MyMessage dimmerMsg(0, V_DIMMER);
MyMessage lightMsg(0, V_LIGHT);

#include <SPI.h>
#include "millis.h"
#include "delay.h"
#include "SerialDebug.h"
#include "IthoCC1101.h"
#include "IthoPacket.h"
IthoCC1101 *rf;
void before()
{
  wdt_disable();
  pinMode(LED_PIN,OUTPUT);
  // Startup up the OneWire library
  Serial.println("before(): between wdt_disable and sensors.begin");
  sensors.begin();
  Serial.println("before(): after sensors.begin");
  
  IthoPacket packet;
  millis_t last;
  
  millis_init();
  delay_ms(500);
  
  //system/slave select CC1101
  DigitalPin ss(&PORTC,0);
  
  //set up SPI
  SPITHO spitho(&ss);
  spitho.init();
  
  //init CC1101
  rf(&spitho);
  rf.init();
  
  //set CC1101 registers
  rf.initReceive();
      
  debug.serOut("start\n");
  last = millis();
  sei();
  Serial.println("before(): after cc1101 init,sei()");

}

void setup()
{
  // Pull the gateway's current dim level - restore light level upon node power-up
  request( 0, V_DIMMER );
  // requestTemperatures() will not block current thread
  //sensors.setWaitForConversion(false);
  Serial.print("End setup for node number ");Serial.println(MY_NODE_ID);
}

void presentation() {
  // Send the sketch version information to the gateway and Controller
  //Serial.print("Start presentation for node number ");Serial.print(MY_NODE_ID);
  sendSketchInfo("FanAttic_156", "1.1");

  // Fetch the number of attached temperature sensors
  numSensors=1; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!numSensors = sensors.getDeviceCount();


  present(7,S_DIMMER);
  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     present(i+1, S_TEMP);
  }
}

void loopxxx() {
  //wdt_reset();
  Serial.println("Start loop");
  float temperature=37.9;
  sensors.requestTemperatures(); // Send the command to get temperatures
Serial.print("Temperature for Device 1 is: ");
  temperature=sensors.getTempCByIndex(0); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
Serial.print(temperature);
  send (msg.setSensor(0).set(temperature,1));
  Serial.println("msg sent");
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  //sleep(SLEEP_TIME);
  wait(conversionTime);
  wait(SLEEP_TIME,C_SET,V_PERCENTAGE);
  Serial.println("End loop");
  analogWrite(LED_PIN,(int)128);
}
void receive(const MyMessage &message) {

    Serial.print( "Message received" );
  if (message.getType() == V_LIGHT || message.getType() == V_DIMMER || message.getType() == V_PERCENTAGE) {

    //  Retrieve the power or dim level from the incoming request message
    int requestedLevel = atoi( message.data );

    // Adjust incoming level if this is a V_LIGHT variable update [0 == off, 1 == on]
    requestedLevel *= ( message.getType() == V_LIGHT ? 100 : 1 );

    // Clip incoming level to valid range of 0 to 100
    requestedLevel = requestedLevel > 100 ? 100 : requestedLevel;
    requestedLevel = requestedLevel < 0   ? 0   : requestedLevel;

    #ifdef MY_DEBUG

    Serial.print( "Changing level to " );
    Serial.print( requestedLevel );
    Serial.print( ", from " );
    Serial.println( currentLevel );
    #endif

    fadeToLevel( requestedLevel );

    // Inform the gateway of the current DimmableLED's SwitchPower1 and LoadLevelStatus value...
    send(lightMsg.set(currentLevel > 0));

    // hek comment: Is this really nessesary?
    send( dimmerMsg.set(currentLevel) );
  }
}
/***
 *  This method provides a graceful fade up/down effect
 */
void fadeToLevel( int toLevel )
{

  int delta = ( toLevel - currentLevel ) < 0 ? -1 : 1;

  while ( currentLevel != toLevel ) {
    currentLevel += delta;
    analogWrite( LED_PIN, (int)(currentLevel / 100. * 255) );
    delay( FADE_DELAY );
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


    if (rf.checkForNewPacket())
    {
      packet = rf.getLastPacket();
      
      //show counter
      debug.serOut("counter=");
      debug.serOutInt(packet.counter);
      debug.serOut(", ");
      
      //show command
      switch (packet.command)
      {
        case unknown:
          debug.serOut("unknown\n");
          break;
        case rft_fullpower:
          debug.serOut("fullpower\n");
          break;
        case rft_standby:
        case duco_standby:
          debug.serOut("standby\n");
          break;
        case rft_low:
        case duco_low:
          debug.serOut("low\n");
          break;
        case rft_medium:
        case duco_medium:
          debug.serOut("medium\n");
          break;
        case rft_high:
        case duco_high:
          debug.serOut("high\n");
          break;
        case rft_timer1:
          debug.serOut("timer1\n");
          break;
        case rft_timer2:
          debug.serOut("timer2\n");
          break;
        case rft_timer3:
          debug.serOut("timer3\n");
          break;
        case join:
          debug.serOut("join\n");
          break;
        case leave:
          debug.serOut("leave\n");
          break;
      }
        
    }
    
    if (millis() - last > 15000)
    {
      last = millis();
      rf.sendCommand(rft_fullpower);
      rf.initReceive();
    }

  
  
  // sleep with time is difficult
  delay(SLEEP_TIME);
}
