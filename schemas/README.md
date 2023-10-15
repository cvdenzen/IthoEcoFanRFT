WORK IN PROGRESS june 2021
mysensors_cc1101
This is the device that will control the home fan.
Software for Arduino Pro Mini in another github repo:
https://github.com/cvdenzen/IthoEcoFanRFT.git
Features:
- MySensors receiver RFM69
- cc1101 sender
- hot water tube temperature sensor (ntc)

Connections

Arduino pro mini 3.3V 8MHz

ftdi header:
gnd cts vcc rx0 tx0 dtr


           D1 - tx0 | raw
           D0 - rxi | gnd
                rst | rst
                gnd | vcc
RFM69:DIO0!D2 - 2   | a3 - D17 - ADC3 - ntc Sensor
CC11:GD02 !D3 - 3   | a2 - D16 - ADC2 - ntc reference input (resistor divider)
LED:RM     D4 - 4   | a1 - D15 - ADC0 - digital out: ntc sensor power
LED:G1     D5 - 5   | A0 - D14 - PC0 - CC11:CSN XXXXXXXXXXXXXX moved to D9-9
LED:G2     D6 - 6   | 13 - D13 - PB5 - SCK  RFM69:SCK  CC11:SCK
CNTCT      D7 - 7   | 12 - D12 - PB4 - MISO RFM69:MISO CC11:MISO
ONEWIRE    D8 - 8   | 11 - D11 - PB3 - MOSI RFM69:MOSI CC11:MOSI
CC11:CSN   D9 - 9   | 10 - D10 - PB2 - SS   RFM69:NSS

A5
A4
A7
A6

RFM69: RFM69 transceiver
CC11: cc1101 transceiver
LED:RM: red (or amber) led, indicating manually controlled fan
LED:G1: green led, indicating speed 1
LED:G2: green led, indicating speed 2
CNTCT: Switch to control fan to manual (cycle: auto-manOff-manSpeed1-manSpeed2)
ONEWIRE: For e.g. temp sensor, needs 4k7 pull-up

cc1101: https://www.ti.com/lit/ds/symlink/cc1101.pdf?ts=1624529600954&ref_url=https%253A%252F%252Fwww.google.com%252F
spi clock idle is 0
https://www.arduino.cc/en/reference/SPI
arduino SPI_MODE0 or SPI_MODE1
rising edge is active, so no valid mode according to arduino
https://github.com/supersjimmie/IthoEcoFanRFT

Connections between the CC1101 and the ESP8266 or Arduino:
CC11xx pins    ESP pins Arduino pins  Description
*  1 - VCC        VCC      VCC           3v3
*  2 - GND        GND      GND           Ground
*  3 - MOSI       13=D7    Pin 11        Data input to CC11xx
*  4 - SCK        14=D5    Pin 13        Clock pin
*  5 - MISO/GDO1  12=D6    Pin 12        Data output from CC11xx / serial clock from CC11xx
*  6 - GDO2       04=D2    Pin  2        Programmable output
*  7 - GDO0       ?        Pin  ?        Programmable output
*  8 - CSN        15=D8    Pin 10        Chip select / (SPI_SS)

Maybe RFM69:DIO0 and CC11:GD02 should be changed, depending on library possibilities
Mysensors looks simple to configure this IRQ pin, so pin 3 should be nice.

Power consumption is not a problem, as the unit should always listen to the MySensors network to
receive commands to control the fan (e.g. because the pir sensor in the bathroom detected motion).

ntc sensor for hot water sensing: 3 resistors, ntc is 47k at 20 degrees C.

Voeding: old usb adapter 5V. On board 3.3 V low drop stabilizer.