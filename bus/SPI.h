/*
 * Author: Klusjesman
 */

#ifndef __SPITHO_H__
#define __SPITHO_H__

//#include <stdio.h>
#include "../component/DigitalPin.h"

/*	SPITHO ports and pins configuration */
#define SPITHO_PORT_MOSI PORTB		//PB3 = MOSI
#define SPITHO_PIN_MOSI 3
#define SPITHO_DDR_MOSI DDRB

#define SPITHO_PORT_MISO PORTB		//PB4 = MISO
#define SPITHO_PIN_MISO 4
#define SPITHO_DDR_MISO DDRB

#define SPITHO_PORT_SCK PORTB		//PB5 = SCK
#define SPITHO_PIN_SCK 5
#define SPITHO_DDR_SCK DDRB


class SPITHO
{
	//variables
	public:
	protected:
	private:
		DigitalPin *ssPin;

	//functions
	public:
		SPITHO(DigitalPin *ssPin);
		~SPITHO();
		
		void init();
		void waitMiso();
		
		void select();
		void deselect();
		
		uint8_t write(uint8_t value);
		uint8_t read();
		
		void attachInterrupt();
		void detachInterrupt();
		
	protected:
	private:
		SPITHO();
		SPITHO( const SPITHO &c );
		SPITHO& operator=( const SPITHO &c );

}; //SPI

SPITHO *spi;
#endif //__SPITHO_H__
