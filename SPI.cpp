/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "SPI.h"

SPIClass SPI;

uint8_t SPIClass::initialized = 0;
uint8_t SPIClass::interruptMode = 0;
uint8_t SPIClass::interruptMask = 0;
uint8_t SPIClass::interruptSave = 0;
#ifdef SPI_TRANSACTION_MISMATCH_LED
uint8_t SPIClass::inTransactionFlag = 0;
#endif

void SPIClass::begin()
{
	
 // uint8_t sreg = SREG;
//  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  if (!initialized) {
		//We don't need Chip Select for our application
    // Set SS to high so a connected chip will be "deselected" by default
    //uint8_t port = digitalPinToPort(SS);
    //uint8_t bit = digitalPinToBitMask(SS);
    //volatile uint8_t *reg = portModeRegister(port);

    // if the SS pin is not already configured as an output
    // then set it high (to enable the internal pull-up resistor)
    //if(!(*reg & bit)){
    //  digitalWrite(SS, HIGH);
    //}

    // When the SS pin is set as OUTPUT, it can be used as
    // a general purpose output port (it doesn't influence
    // SPI operations).
    //pinMode(SS, OUTPUT);
		

    // Warning: if the SS pin ever becomes a LOW INPUT then SPI
    // automatically switches to Slave, so the data direction of
    // the SS pin MUST be kept as OUTPUT.
    //SPCR |= _BV(MSTR); // set as master
    //SPCR |= _BV(SPE);	 //enable SPI

    // Set direction register for SCK and MOSI pin.
    // MISO pin automatically overrides to INPUT.
    // By doing this AFTER enabling SPI, we avoid accidentally
    // clocking in a single bit since the lines go directly
    // from "input" to SPI control.
    // http://code.google.com/p/arduino/issues/detail?id=888
		
				
		
		//*********************************************************************************
		
		
		

			
		int SCK = 13;
		int MOSI = 15;
		int MISO = 14;
//		int NSS = 12;
		//pinMode(GPIOE, NSS, ALT_FUNC);
    pinMode(GPIOE, SCK, ALT_FUNC);
    pinMode(GPIOE, MOSI, ALT_FUNC);
		pinMode(GPIOE, MISO, ALT_FUNC);
		//pinAlternateFunction(GPIOE, NSS, 5);
		pinAlternateFunction(GPIOE, SCK, 5); //AF 5 = SPI
    pinAlternateFunction(GPIOE, MOSI, 5);
		pinAlternateFunction(GPIOE, MISO, 5);
		
		//enable clock for SPI1
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

		SPI1->CR1 &= ~SPI_CR1_SPE; // Disable SPI
		
		// Configure duplex or receive-only
		// 0 = Full duplex (transmit and receive), 1 = Receive-only
		SPI1->CR1 &= ~SPI_CR1_RXONLY;
			
		// Bidirectional data mode enable: This bit enables half-duplex
		// communication using common single bidirectional data line.
		// 0 = 2-line unidirectional data mode selected
		// 1 = 1-line bidirectional data mode selected
		SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
			
		// Output enable in bidirectional mode
		// 0 = Output disable (receive-only mode)
		// 1 = Output enabled (transmit-only mode)
		SPI1->CR1 &= ~SPI_CR1_BIDIOE;
		
		//set data size
		SPI1->CR2 &= ~SPI_CR2_DS;
		SPI1->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2; //0111 8BIT
		
		// Bit order
		// 0 = MSB Transmitted/recieved first
		// 1 = LSB transmitted/received first
		SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
		
		// clock phase
		// 0 = the first clock transition is the first data capture edge
		// 1 = the second clock transition is the first data caputre edge
		SPI1->CR1 &= ~SPI_CR1_CPHA; // 1st edge
		
		// clock polarity
		// 0 = set CK to 0 when idle
		// 1 = set CK to 1 when idle
		SPI1->CR1 &= ~SPI_CR1_CPOL; // Polarity Low
		
		
		
		//CRC Polynomial
		SPI1->CRCPR = 10;
		
		// Hardware CRC calculation disabled
		SPI1->CR1 &= ~SPI_CR1_CRCEN;
		
		// Frame format: 0 = SPI Motorola mode, 1 = SPI TI mode
		SPI1->CR2 &= ~SPI_CR2_FRF;
		
		//NSSGPIO: 	The value of SSI is forced onto the NSS pin and the IO value
		// of the NSS pin is ignored.
		// 1 = Software slabe management enabled
		// 0 = Hardware NSS management enabled
		//SPI1->CR1 |= SPI_CR1_SSM;
		
		// Set as Master: 0 = slave, 1 = master
		SPI1->CR1 |= SPI_CR1_MSTR;
		
		// Manage NSS (slave selection) by using software
		SPI1->CR1 |= SPI_CR1_SSI;
		
		// Enable NSS pulse management 
		SPI1->CR2 |= SPI_CR2_NSSP;
		
		//recieve buffer not empty
		SPI1->CR2 |= SPI_CR2_FRXTH;
		
		//set speed
		SPI1->CR1 |= SPI_CR1_BR_0;
		SPI1->CR1 |= SPI_CR1_BR_1;
		


		//enable SPI
		SPI1->CR1 |= SPI_CR1_SPE;
		
		
		//*********************************************************************************
		
		
		
		
			
  }
  initialized++; // reference count
  //SREG = sreg;
}

void SPIClass::end() {
  //uint8_t sreg = SREG;
//  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  // Decrease the reference counter
  if (initialized)
    initialized--;
  // If there are no more references disable SPI
  if (!initialized) {
    //SPCR &= ~_BV(SPE);
		SPI1->CR1 &= ~SPI_CR1_SPE;
    interruptMode = 0;
    #ifdef SPI_TRANSACTION_MISMATCH_LED
    inTransactionFlag = 0;
    #endif
  }
  //SREG = sreg;
}
/*
// mapping of interrupt numbers to bits within SPI_AVR_EIMSK
#if defined(__AVR_ATmega32U4__)
  #define SPI_INT0_MASK  (1<<INT0)
  #define SPI_INT1_MASK  (1<<INT1)
  #define SPI_INT2_MASK  (1<<INT2)
  #define SPI_INT3_MASK  (1<<INT3)
  #define SPI_INT4_MASK  (1<<INT6)
#elif defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__)
  #define SPI_INT0_MASK  (1<<INT0)
  #define SPI_INT1_MASK  (1<<INT1)
  #define SPI_INT2_MASK  (1<<INT2)
  #define SPI_INT3_MASK  (1<<INT3)
  #define SPI_INT4_MASK  (1<<INT4)
  #define SPI_INT5_MASK  (1<<INT5)
  #define SPI_INT6_MASK  (1<<INT6)
  #define SPI_INT7_MASK  (1<<INT7)
#elif defined(EICRA) && defined(EICRB) && defined(EIMSK)
  #define SPI_INT0_MASK  (1<<INT4)
  #define SPI_INT1_MASK  (1<<INT5)
  #define SPI_INT2_MASK  (1<<INT0)
  #define SPI_INT3_MASK  (1<<INT1)
  #define SPI_INT4_MASK  (1<<INT2)
  #define SPI_INT5_MASK  (1<<INT3)
  #define SPI_INT6_MASK  (1<<INT6)
  #define SPI_INT7_MASK  (1<<INT7)
#else
  #ifdef INT0
  #define SPI_INT0_MASK  (1<<INT0)
  #endif
  #ifdef INT1
  #define SPI_INT1_MASK  (1<<INT1)
  #endif
  #ifdef INT2
  #define SPI_INT2_MASK  (1<<INT2)
  #endif
#endif

void SPIClass::usingInterrupt(uint8_t interruptNumber)
{
  uint8_t mask = 0;
  uint8_t sreg = SREG;
  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  switch (interruptNumber) {
  #ifdef SPI_INT0_MASK
  case 0: mask = SPI_INT0_MASK; break;
  #endif
  #ifdef SPI_INT1_MASK
  case 1: mask = SPI_INT1_MASK; break;
  #endif
  #ifdef SPI_INT2_MASK
  case 2: mask = SPI_INT2_MASK; break;
  #endif
  #ifdef SPI_INT3_MASK
  case 3: mask = SPI_INT3_MASK; break;
  #endif
  #ifdef SPI_INT4_MASK
  case 4: mask = SPI_INT4_MASK; break;
  #endif
  #ifdef SPI_INT5_MASK
  case 5: mask = SPI_INT5_MASK; break;
  #endif
  #ifdef SPI_INT6_MASK
  case 6: mask = SPI_INT6_MASK; break;
  #endif
  #ifdef SPI_INT7_MASK
  case 7: mask = SPI_INT7_MASK; break;
  #endif
  default:
    interruptMode = 2;
    break;
  }
  interruptMask |= mask;
  if (!interruptMode)
    interruptMode = 1;
  SREG = sreg;
}

void SPIClass::notUsingInterrupt(uint8_t interruptNumber)
{
  // Once in mode 2 we can't go back to 0 without a proper reference count
  if (interruptMode == 2)
    return;
  uint8_t mask = 0;
  uint8_t sreg = SREG;
  noInterrupts(); // Protect from a scheduler and prevent transactionBegin
  switch (interruptNumber) {
  #ifdef SPI_INT0_MASK
  case 0: mask = SPI_INT0_MASK; break;
  #endif
  #ifdef SPI_INT1_MASK
  case 1: mask = SPI_INT1_MASK; break;
  #endif
  #ifdef SPI_INT2_MASK
  case 2: mask = SPI_INT2_MASK; break;
  #endif
  #ifdef SPI_INT3_MASK
  case 3: mask = SPI_INT3_MASK; break;
  #endif
  #ifdef SPI_INT4_MASK
  case 4: mask = SPI_INT4_MASK; break;
  #endif
  #ifdef SPI_INT5_MASK
  case 5: mask = SPI_INT5_MASK; break;
  #endif
  #ifdef SPI_INT6_MASK
  case 6: mask = SPI_INT6_MASK; break;
  #endif
  #ifdef SPI_INT7_MASK
  case 7: mask = SPI_INT7_MASK; break;
  #endif
  default:
    break;
    // this case can't be reached
  }
  interruptMask &= ~mask;
  if (!interruptMask)
    interruptMode = 0;
  SREG = sreg;
}*/
