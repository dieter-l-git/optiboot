/*
	Author:		Dieter Lambrecht
	Email:		Dieter.l@gmx.de
	Date:		06-09-2016
	Project:		Duino ATmega32 for Arduino IDE
	Version:		v1.0
	*/

/*
	Copyright (c) 2016 Dieter Lambrecht

	based on original pins_arduino.h

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
	files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
	modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
	is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
	LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
	IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
	*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

/* Pin-Numbers here are defined below as DXX*/
/* Example:	PIN_SPI_SS	= D31 -> (31) */
#define PIN_SPI_SS    (31)
#define PIN_SPI_MOSI  (28)
#define PIN_SPI_MISO  (29)
#define PIN_SPI_SCK   (30)

static const uint8_t SS = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (13)
#define PIN_WIRE_SCL        (12)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define NUM_ANALOG_INPUTS           6
#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 22 : -1)

#define digitalPinHasPWM(p)         ((p) == 3 || (p) == 7 || (p) == 10 || (p) == 11)

#define LED_BUILTIN 10

#define PIN_A0   (27)
#define PIN_A1   (26)
#define PIN_A2   (25)
#define PIN_A3   (24)
#define PIN_A4   (23)
#define PIN_A5   (22)
#define PIN_A6   (21)
#define PIN_A7   (20)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;

#ifdef ARDUINO_MAIN

#define PA 1
#define PB 2
#define PC 3
#define PD 4

/*
	Duino Mega32  -  ATMEL ATmega32 TQPF44

	Func		PORT		Pin		Duino	2nd
	--------------------------------------------
	(MOSI)	PB5		1		D28		ISCP-MOSI
	(MISO)	PB6		2		D29		ISCP-MISO
	(SCK)	PB7		3		D30		ICSP-SCK
	RESET			4
	VCC				5
	GND				6
	XTAL1			7
	XTAL2			8
	(RXD)	PD0		9		D0			RxD
	(TXD)	PD1		10		D1			TxD
	(INT0)	PD2		11		D2
	(INT1)	PD3		12		D9
	(OC1B)	PD4		13		D3
	(OC1A)	PD5		14		D11
	(ICP1)	PD6		15		D8
	(OC2)	PD7		16		D10
	VCC				17
	GND				18
	(SCL)	PC0		19		D12		SCL
	(SDA)	PC1		20		D13		SDA
	(TCK)	PC2		21		D14		NC
	(TMS)	PC3		22		D15		NC
	(TDO)	PC4		23		D16		NC
	(TDI)	PC5		24		D17		NC
	(TOSC1)	PC6		25		D18		NC
	(TOSC2)	PC7		26		D19		NC
	AVCC			27
	AGND			28
	AREF			29		AREF
	(ADC7)	PA7		30		D20		NC
	(ADC6)	PA6		31		D21		NC
	(ADC5)	PA5		32		D22		A5
	(ADC4)	PA4		33		D23		A4
	(ADC3)	PA3		34		D24		A3
	(ADC2)	PA2		35		D25		A2
	(ADC1)	PA1		36		D26		A1
	(ADC0)	PA0		37		D27		A0
	VCC				38
	GND				39
	(XCK)	PB0		40		D4
	(T1)	PB1		41		D5
	(INT2)	PB2		42		D6
	(OC0)	PB3		43		D7
	(SS)	PB4		44		D31
	*/

const uint16_t PROGMEM port_to_mode_PGM[5] = {
	NOT_A_PORT,
	(uint16_t)&DDRA,
	(uint16_t)&DDRB,
	(uint16_t)&DDRC,
	(uint16_t)&DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[5] = {
	NOT_A_PORT,
	(uint16_t)&PORTA,
	(uint16_t)&PORTB,
	(uint16_t)&PORTC,
	(uint16_t)&PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[5] = {
	NOT_A_PIN,
	(uint16_t)&PINA,
	(uint16_t)&PINB,
	(uint16_t)&PINC,
	(uint16_t)&PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[32] = {
	PD,  // PD0 ** D0
	PD,  // PD1 ** D1
	PD,  // PD2 ** D2
	PD,  // PD4 ** D3
	PB,  // PB0 ** D4
	PB,  // PB1 ** D5
	PB,  // PB2 ** D6
	PB,  // PB3 ** D7
	PD,  // PD6 ** D8
	PD,  // PD3 ** D9
	PD,  // PD7 ** D10
	PD,  // PD5 ** D11
	PC,  // PC0 ** D12
	PC,  // PC1 ** D13
	PC,  // PC2 ** D14
	PD,  // PC3 ** D15
	PC,  // PC4 ** D16
	PC,  // PC5 ** D17
	PC,  // PC6 ** D18
	PC,  // PC7 ** D19
	PA,  // PA7 ** D20
	PA,  // PA6 ** D21
	PA,  // PA5 ** A5 D22
	PA,  // PA4 ** A4 D23
	PA,  // PA3 ** A3 D24
	PA,  // PA2 ** A2 D25
	PA,  // PA1 ** A1 D26
	PA,  // PA0 ** A0 D27
	PB,  // PB5 ** D28
	PB,  // PB6 ** D29
	PB,  // PB7 ** D30
	PB,  // PB4 ** D31
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[32] = {
	_BV(0),  // PD0 ** D0
	_BV(1),  // PD1 ** D1
	_BV(2),  // PD2 ** D2
	_BV(4),  // PD4 ** D3
	_BV(0),  // PB0 ** D4
	_BV(1),  // PB1 ** D5
	_BV(2),  // PB2 ** D6
	_BV(3),  // PB3 ** D7
	_BV(6),  // PD6 ** D8
	_BV(3),  // PD3 ** D9
	_BV(7),  // PD7 ** D10
	_BV(5),  // PD5 ** D11
	_BV(0),  // PC0 ** D12
	_BV(1),  // PC1 ** D13
	_BV(2),  // PC2 ** D14
	_BV(3),  // PC3 ** D15
	_BV(4),  // PC4 ** D16
	_BV(5),  // PC5 ** D17
	_BV(6),  // PC6 ** D18
	_BV(7),  // PC7 ** D19
	_BV(7),  // PA7 ** D20
	_BV(6),  // PA6 ** D21
	_BV(5),  // PA5 ** A5 D22
	_BV(4),  // PA4 ** A4 D23
	_BV(3),  // PA3 ** A3 D24
	_BV(2),  // PA2 ** A2 D25
	_BV(1),  // PA1 ** A1 D26
	_BV(0),  // PA0 ** A0 D27
	_BV(5),  // PB5 ** D28
	_BV(6),  // PB6 ** D29
	_BV(7),  // PB7 ** D30
	_BV(4),  // PB4 ** D31
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[32] = {
	NOT_ON_TIMER,	// PD0 ** D0
	NOT_ON_TIMER,	// PD1 ** D1
	NOT_ON_TIMER,	// PD2 ** D2
	TIMER1B,		// PD4 ** D3
	NOT_ON_TIMER,	// PB0 ** D4
	NOT_ON_TIMER,	// PB1 ** D5
	NOT_ON_TIMER,	// PB2 ** D6
	TIMER0A,		// PB3 ** D7
	NOT_ON_TIMER,	// PD6 ** D8
	NOT_ON_TIMER,	// PD3 ** D9
	TIMER2,			// PD7 ** D10
	TIMER1A,		// PD5 ** D11
	NOT_ON_TIMER,	// PC0 ** D12
	NOT_ON_TIMER,	// PC1 ** D13
	NOT_ON_TIMER,	// PC2 ** D14
	NOT_ON_TIMER,	// PC3 ** D15
	NOT_ON_TIMER,	// PC4 ** D16
	NOT_ON_TIMER,	// PC5 ** D17
	NOT_ON_TIMER,	// PC6 ** D18
	NOT_ON_TIMER,	// PC7 ** D19
	NOT_ON_TIMER,	// PA7 ** D20
	NOT_ON_TIMER,	// PA6 ** D21
	NOT_ON_TIMER,	// PA5 ** A5 D22
	NOT_ON_TIMER,	// PA4 ** A4 D23
	NOT_ON_TIMER,	// PA3 ** A3 D24
	NOT_ON_TIMER,	// PA2 ** A2 D25
	NOT_ON_TIMER,	// PA1 ** A1 D26
	NOT_ON_TIMER,	// PA0 ** A0 D27
	NOT_ON_TIMER,	// PB5 ** D28
	NOT_ON_TIMER,	// PB6 ** D29
	NOT_ON_TIMER,	// PB7 ** D30
	NOT_ON_TIMER,	// PB4 ** D31
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
#define SERIAL_PORT_MONITOR   Serial
#define SERIAL_PORT_HARDWARE  Serial

#endif
