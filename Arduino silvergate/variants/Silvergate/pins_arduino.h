/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/


#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            54
#define NUM_ANALOG_INPUTS           8
#define analogInputToDigitalPin(p)  ((p < NUM_ANALOG_INPUTS) ? (p) + 46 : -1)
#define digitalPinHasPWM(p)         (((p) >= 2 && (p) <= 9) 

#define PIN_SPI_SS    (16)
#define PIN_SPI_MOSI  (11)
#define PIN_SPI_MISO  (12)
#define PIN_SPI_SCK   (10)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (18)
#define PIN_WIRE_SCL        (17)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define LED_BUILTIN 26

#define PIN_A0   (46)
#define PIN_A1   (47)
#define PIN_A2   (48)
#define PIN_A3   (49)
#define PIN_A4   (50)
#define PIN_A5   (51)
#define PIN_A6   (52)
#define PIN_A7   (53)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;

// A majority of the pins are NOT PCINTs, SO BE WARNED (i.e. you cannot use them as receive pins)
// Only pins available for RECEIVE (TRANSMIT can be on any pin):
// (I've deliberately left out pin mapping to the Hardware USARTs - seems senseless to me)
// Pins: 10, 11, 12, 13,  50, 51, 52, 53,  62, 63, 64, 65, 66, 67, 68, 69

#define digitalPinToPCICR(p)    ( (((p) >= 6) && ((p) <= 12)) || (((p) == 16) ? (&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p) ( (((p) >= 6) && ((p) <= 12)) || (((p) == 16) ? 0 : 0 ) )

#define digitalPinToPCMSK(p)    ( (((p) >= 6) && ((p) <= 12)) || (((p) == 16) ? (&PCMSK0) : ((uint8_t *)0) ) )

#define digitalPinToPCMSKbit(p) (( ((p) == 6) ? 4 : ( ((p)  == 7) ? 5 : ( ((p)  == 8) ? 6 : ( ((p)  == 9) ? 7 : ( ((p) == 10) ? 1 : ( ((p) == 11) ? 2 : ( ((p) == 12) ? 3 :  ( ((p) == 16) ? 0 :  0 )))))))))

//#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) >= 18 && (p) <= 21 ? 23 - (p) : NOT_AN_INTERRUPT)))

#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
	(uint16_t) &DDRG,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
	(uint16_t) &PORTG,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
	(uint16_t) &PING,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	// -------------------------------------------		
	PE	, // PE 0 ** 0 ** USART0_RX	
	PE	, // PE 1 ** 1 ** USART0_TX	
	PE	, // PE 4 ** 2 ** TIMER3B 	
	PE	, // PE 5 ** 3 ** TIMER3C 	
	PG	, // PG 5 ** 4 ** TIMER0B	
	PE	, // PE 3 ** 5 ** TIMER3A 	
	PB	, // PB 4 ** 6 ** TIMER2A 	
	PB	, // PB 5 ** 7 ** TIMER1A 	
	PB	, // PB 6 ** 8 ** TIMER1B 	
	PB	, // PB 7 ** 9 ** TIMER0A 	
	PB	, // PB 1 ** 10 ** SCK	
	PB	, // PB 2 ** 11 ** MOSI	
	PB	, // PB 3 ** 12 ** MISO	
	PE	, // PE 2 ** 13 ** D13	
	PE	, // PE 6 ** 14 ** D14	
	PE	, // PE 7 ** 15 ** D15	
	PB	, // PB 0 ** 16 ** SS	
	PD	, // PD 0 ** 17 ** SCL	
	PD	, // PD 1 ** 18 ** SDA	
	PD	, // PD 2 ** 19 ** USART1_RX	
	PD	, // PD 3 ** 20 ** USART1_TX	
	PD	, // PD 4 ** 21 ** D21	
	PD	, // PD 5 ** 22 ** D22	
	PD	, // PD 6 ** 23 ** D23	
	PD	, // PD 7 ** 24 ** D24	
	PG	, // PG 0 ** 25 ** D25	
	PG	, // PG 1 ** 26 ** D26	
	PG	, // PG 2 ** 27 ** D27	
	PG	, // PG 3 ** 28 ** D28	
	PG	, // PG 4 ** 29 ** D29	
	PC	, // PC 0 ** 30 ** D30	
	PC	, // PC 1 ** 31 ** D31	
	PC	, // PC 2 ** 32 ** D32	
	PC	, // PC 3 ** 33 ** D33	
	PC	, // PC 4 ** 34 ** D34	
	PC	, // PC 5 ** 35 ** D35	
	PC	, // PC 6 ** 36 ** D36	
	PC	, // PC 7 ** 37 ** D37	
	PA	, // PA 0 ** 38 ** D38	
	PA	, // PA 1 ** 39 ** D39	
	PA	, // PA 2 ** 40 ** D40	
	PA	, // PA 3 ** 41 ** D41	
	PA	, // PA 4 ** 42 ** D42	
	PA	, // PA 5 ** 43 ** D43	
	PA	, // PA 6 ** 44 ** D44	
	PA	, // PA 7 ** 45 ** D45	
	PF	, // PF 0 ** 46 ** A0	
	PF	, // PF 1 ** 47 ** A1	
	PF	, // PF 2 ** 48 ** A2	
	PF	, // PF 3 ** 49 ** A3	
	PF	, // PF 4 ** 50 ** A4	
	PF	, // PF 5 ** 51 ** A5	
	PF	, // PF 6 ** 52 ** A6	
	PF	, // PF 7 ** 53 ** A7	
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	_BV( 0 )	, // PE 0 ** 0 ** USART0_RX	
	_BV( 1 )	, // PE 1 ** 1 ** USART0_TX	
	_BV( 4 )	, // PE 4 ** 2 ** TIMER3B  	
	_BV( 5 )	, // PE 5 ** 3 ** TIMER3C 	
	_BV( 5 )	, // PG 5 ** 4 ** TIMER0B 	
	_BV( 3 )	, // PE 3 ** 5 ** TIMER3A 	
	_BV( 4 )	, // PB 4 ** 6 ** TIMER2A 	
	_BV( 5 )	, // PB 5 ** 7 ** TIMER1A 	
	_BV( 6 )	, // PB 6 ** 8 ** TIMER1B 	
	_BV( 7 )	, // PB 7 ** 9 ** TIMER0A 	
	_BV( 1 )	, // PB 1 ** 10 ** SCK	
	_BV( 2 )	, // PB 2 ** 11 ** MOSI	
	_BV( 3 )	, // PB 3 ** 12 ** MISO 	
	_BV( 2 )	, // PE 2 ** 13 ** D13	
	_BV( 6 )	, // PE 6 ** 14 ** D14	
	_BV( 7 )	, // PE 7 ** 15 ** D15
	_BV( 0 )	, // PB 0 ** 16 ** SS	
	_BV( 0 )	, // PD 0 ** 17 ** SCL	
	_BV( 1 )	, // PD 1 ** 18 ** SDA	
	_BV( 2 )	, // PD 2 ** 19 ** USART1_RX	
	_BV( 3 )	, // PD 3 ** 20 ** USART1_TX	
	_BV( 4 )	, // PD 4 ** 21 ** D21	
	_BV( 5 )	, // PD 5 ** 22 ** D22	
	_BV( 6 )	, // PD 6 ** 23 ** D23	
	_BV( 7 )	, // PD 7 ** 24 ** D24	
	_BV( 0 )	, // PG 0 ** 25 ** D25	
	_BV( 1 )	, // PG 1 ** 26 ** D26	
	_BV( 2 )	, // PG 2 ** 27 ** D27	
	_BV( 3 )	, // PG 3 ** 28 ** D28	
	_BV( 4 )	, // PG 4 ** 29 ** D29	
	_BV( 0 )	, // PC 0 ** 30 ** D30	
	_BV( 1 )	, // PC 1 ** 31 ** D31	
	_BV( 2 )	, // PC 2 ** 32 ** D32	
	_BV( 3 )	, // PC 3 ** 33 ** D33	
	_BV( 4 )	, // PC 4 ** 34 ** D34	
	_BV( 5 )	, // PC 5 ** 35 ** D35	
	_BV( 6 )	, // PC 6 ** 36 ** D36	
	_BV( 7 )	, // PC 7 ** 37 ** D37	
	_BV( 0 )	, // PA 0 ** 38 ** D38	
	_BV( 1 )	, // PA 1 ** 39 ** D39	
	_BV( 2 )	, // PA 2 ** 40 ** D40	
	_BV( 3 )	, // PA 3 ** 41 ** D41	
	_BV( 4 )	, // PA 4 ** 42 ** D42	
	_BV( 5 )	, // PA 5 ** 43 ** D43	
	_BV( 6 )	, // PA 6 ** 44 ** D44	
	_BV( 7 )	, // PA 7 ** 45 ** D45	
	_BV( 0 )	, // PF 0 ** 46 ** A0	
	_BV( 1 )	, // PF 1 ** 47 ** A1	
	_BV( 2 )	, // PF 2 ** 48 ** A2	
	_BV( 3 )	, // PF 3 ** 49 ** A3	
	_BV( 4 )	, // PF 4 ** 50 ** A4	
	_BV( 5 )	, // PF 5 ** 51 ** A5	
	_BV( 6 )	, // PF 6 ** 52 ** A6	
	_BV( 7 )	, // PF 7 ** 53 ** A7	
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	// TIMERS		
	// -------------------------------------------		
	NOT_ON_TIMER, // PE 0 ** 0 ** USART0_RX	
	NOT_ON_TIMER, // PE 1 ** 1 ** USART0_TX	
	TIMER3B,      // PE 4 ** 2 ** TIMER3B	
	TIMER3C, 	  // PE 5 ** 3 ** TIMER3C	
	TIMER0B, 	  // PG 5 ** 4 ** TIMER0B 	
	TIMER3A,   	  // PE 3 ** 5 ** TIMER3A 	
	TIMER2A, 	  // PB 4 ** 6 ** TIMER2A 	
	TIMER1A, 	  // PB 5 ** 7 ** TIMER1A 	
	TIMER1B,      // PB 6 ** 8 ** TIMER1B 	
	TIMER0A, 	  // PB 7 ** 9 ** TIMER0A 	
	NOT_ON_TIMER, // PB 1 ** 10 ** SCK	
	NOT_ON_TIMER, // PB 2 ** 11 ** MOSI	
	NOT_ON_TIMER, // PB 3 ** 12 ** MISO	
	NOT_ON_TIMER, // PE 2 ** 13 ** D13 	
	NOT_ON_TIMER, // PE 6 ** 14 ** D14 	
	NOT_ON_TIMER, // PE 7 ** 15 ** D15 
	NOT_ON_TIMER, // PB 0 ** 16 ** SS	
	NOT_ON_TIMER, // PD 0 ** 17 ** SCL	
	NOT_ON_TIMER, // PD 1 ** 18 ** SDA	
	NOT_ON_TIMER, // PD 2 ** 19 ** USART1_RX 	
	NOT_ON_TIMER, // PD 3 ** 20 ** USART1_TX 	
	NOT_ON_TIMER, // PD 4 ** 21 ** D21 	
	NOT_ON_TIMER, // PD 5 ** 22 ** D22	
	NOT_ON_TIMER, // PD 6 ** 23 ** D23	
	NOT_ON_TIMER, // PD 7 ** 24 ** D24	
	NOT_ON_TIMER, // PG 0 ** 25 ** D25	
	NOT_ON_TIMER, // PG 1 ** 26 ** D26	
	NOT_ON_TIMER, // PG 2 ** 27 ** D27	
	NOT_ON_TIMER, // PG 3 ** 28 ** D28	
	NOT_ON_TIMER, // PG 4 ** 29 ** D29	
	NOT_ON_TIMER, // PC 0 ** 30 ** D30	
	NOT_ON_TIMER, // PC 1 ** 31 ** D31	
	NOT_ON_TIMER, // PC 2 ** 32 ** D32	
	NOT_ON_TIMER, // PC 3 ** 33 ** D33	
	NOT_ON_TIMER, // PC 4 ** 34 ** D34	
	NOT_ON_TIMER, // PC 5 ** 35 ** D35	
	NOT_ON_TIMER, // PC 6 ** 36 ** D36	
	NOT_ON_TIMER, // PC 7 ** 37 ** D37	
	NOT_ON_TIMER, // PA 0 ** 38 ** D38	
	NOT_ON_TIMER, // PA 1 ** 39 ** D39	
	NOT_ON_TIMER, // PA 2 ** 40 ** D40	
	NOT_ON_TIMER, // PA 3 ** 41 ** D41	
	NOT_ON_TIMER, // PA 4 ** 42 ** D42	
	NOT_ON_TIMER, // PA 5 ** 43 ** D43	
	NOT_ON_TIMER, // PA 6 ** 44 ** D44	
	NOT_ON_TIMER, // PA 7 ** 45 ** D45	
	NOT_ON_TIMER, // PF 0 ** 46 ** A0	
	NOT_ON_TIMER, // PF 1 ** 47 ** A1	
	NOT_ON_TIMER, // PF 2 ** 48 ** A2	
	NOT_ON_TIMER, // PF 3 ** 49 ** A3	
	NOT_ON_TIMER, // PF 4 ** 50 ** A4	
	NOT_ON_TIMER, // PF 5 ** 51 ** A5	
	NOT_ON_TIMER, // PF 6 ** 52 ** A6	
	NOT_ON_TIMER, // PF 7 ** 53 ** A7	
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial


#endif
