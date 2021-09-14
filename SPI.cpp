/*
 * Author: Klusjesman
 */

#include "SPI.h"

// default constructor
SPITHO::SPITHO(DigitalPin *ssPin)
{
	this->ssPin = ssPin;
} //SPI

// default destructor
SPITHO::~SPITHO()
{
} //~SPI

void SPITHO::init()
{
    /* enable outputs for MOSI, SCK, SS, input for MISO */
    SPITHO_DDR_MOSI |= (1 << SPITHO_PIN_MOSI);	//mosi output
    SPITHO_DDR_SCK |= (1 << SPITHO_PIN_SCK);		//sck output
	ssPin->makeOutput();					//ss output
    SPITHO_DDR_MISO &= ~(1 << SPITHO_PIN_MISO);	//miso input
		
    deselect();

    SPCR =	(0 << SPIE) | /* SPITHO Interrupt Enable */
			(1 << SPE)  | /* SPITHO Enable */
			(0 << DORD) | /* Data Order: MSB first */
			(1 << MSTR) | /* Master mode */
			(0 << CPOL) | /* Clock Polarity: SCK low when idle */
			(0 << CPHA) | /* Clock Phase: sample on rising SCK edge */
			(1 << SPR1) | /* Clock Frequency: f_OSC / 128 */
			(1 << SPR0);
    SPSR &= ~(1 << SPI2X); /* No doubled clock frequency */	
	
    deselect();
}

void SPITHO::waitMiso()
{
	while ((SPITHO_PORT_MISO >> SPITHO_PIN_MISO) & 0x01);
}

void SPITHO::select()
{
	ssPin->write(false);
}

void SPITHO::deselect()
{
	ssPin->write(true);
}

uint8_t SPITHO::write(uint8_t value)
{
    SPDR = value;
    /* wait for byte to be shifted out */
    while (!(SPSR & (1 << SPIF)));
    SPSR &= ~(1 << SPIF);
	
	return SPDR;
}

uint8_t SPITHO::read()
{
    SPDR = 0xFF;
    while (!(SPSR & (1 << SPIF)));
    SPSR &= ~(1 << SPIF);

    return SPDR;	
}

void SPITHO::attachInterrupt()
{
	SPCR |= (1<<SPIE);
}

void SPITHO::detachInterrupt()
{
	SPCR &= ~(1<<SPIE);
}
