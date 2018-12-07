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

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include "stm32l476xx.h"
// 00 = input, 01 = Output, 10 = Alternate Function, 11 =  Analog
#define OUTPUT 1
#define INPUT 0
#define ALT_FUNC 2
#define ANALOG 3
void pinMode(GPIO_TypeDef* GPIOx, int pin, int mode);
void pinAlternateFunction(GPIO_TypeDef* GPIOx, int pin, int mode);
void digitalWrite(GPIO_TypeDef* GPIOx, int pin, int value);
int digitalRead(GPIO_TypeDef* GPIOx, int pin);
int millis();

// SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

// SPI_ATOMIC_VERSION means that SPI has atomicity fixes and what version.
// This way when there is a bug fix you can check this define to alert users
// of your code if it uses better version of this library.
// This also implies everything that SPI_HAS_TRANSACTION as documented above is
// available too.
#define SPI_ATOMIC_VERSION 1

// Uncomment this line to add detection of mismatched begin/end transactions.
// A mismatch occurs if other libraries fail to use SPI.endTransaction() for
// each SPI.beginTransaction().  Connect an LED to this pin.  The LED will turn
// on if any mismatch is ever detected.
//#define SPI_TRANSACTION_MISMATCH_LED 5

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

//old - avr
//#define SPI_CLOCK_DIV4 0x00
//#define SPI_CLOCK_DIV16 0x01
//#define SPI_CLOCK_DIV64 0x02
//#define SPI_CLOCK_DIV128 0x03
//#define SPI_CLOCK_DIV2 0x04
//#define SPI_CLOCK_DIV8 0x05
//#define SPI_CLOCK_DIV32 0x06

//new - ARM
#define SPI_CLOCK_DIV_2 0b000;
#define SPI_CLOCK_DIV_4 0b001;
#define SPI_CLOCK_DIV_8 0b010;
#define SPI_CLOCK_DIV_16 0b011;
#define SPI_CLOCK_DIV_32 0b100;
#define SPI_CLOCK_DIV_64 0b101;
#define SPI_CLOCK_DIV_128 0b110;
#define SPI_CLOCK_DIV_256 0b111;


//old - avr
#define SPI_MODE0 0x00
//#define SPI_MODE1 0x04
//#define SPI_MODE2 0x08
//#define SPI_MODE3 0x0C

//#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
//#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
//#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR

// define SPI_AVR_EIMSK for AVR boards with external interrupt pins
//#if defined(EIMSK)
//  #define SPI_AVR_EIMSK  EIMSK
//#elif defined(GICR)
//  #define SPI_AVR_EIMSK  GICR
//#elif defined(GIMSK)
//  #define SPI_AVR_EIMSK  GIMSK
//#endif
/*
class SPISettings {
public:
  SPISettings(uint32_t lock, uint8_t bitOrder, uint8_t dataMode) {
    if (__builtin_constant_p(clock)) {
      init_AlwaysInline(clock, bitOrder, dataMode);
    } else {
      init_MightInline(clock, bitOrder, dataMode);
    }
  }
  SPISettings() {
    init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0);
  }
private:
//  void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
//    init_AlwaysInline(clock, bitOrder, dataMode);
//  }
//  void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
//	__attribute__((__always_inline__)) 
		void init(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
		//arduino - old
    // Clock settings are defined as follows. Note that this shows SPI2X
    // inverted, so the bits form increasing numbers. Also note that
    // fosc/64 appears twice
    // SPR1 SPR0 ~SPI2X Freq
    //   0    0     0   fosc/2
    //   0    0     1   fosc/4
    //   0    1     0   fosc/8
    //   0    1     1   fosc/16
    //   1    0     0   fosc/32
    //   1    0     1   fosc/64
    //   1    1     0   fosc/64
    //   1    1     1   fosc/128

    // We find the fastest clock that is less than or equal to the
    // given clock rate. The clock divider that results in clock_setting
    // is 2 ^^ (clock_div + 1). If nothing is slow enough, we'll use the
    // slowest (128 == 2 ^^ 7, so clock_div = 6).
    uint8_t clockDiv;

    // When the clock is known at compiletime, use this if-then-else
    // cascade, which the compiler knows how to completely optimize
    // away. When clock is not known, use a loop instead, which generates
    // shorter code.
		int F_CPU = 16000000; //clock speed
    
      uint32_t clockSetting = F_CPU / 2;
      clockDiv = 0;
      while (clockDiv < 6 && clock < clockSetting) {
        clockSetting /= 2;
        clockDiv++;
      }
		
		

    //SPE - SPI Enable, MSTR - Master/slave select, DORD (Data Order), 
    // Pack into the SPISettings class
    //spcr = _BV(SPE) | _BV(MSTR) | ((bitOrder == LSBFIRST) ? _BV(DORD) : 0) |
    //  (dataMode & SPI_MODE_MASK) | ((clockDiv >> 1) & SPI_CLOCK_MASK);
    //spsr = clockDiv & SPI_2XCLOCK_MASK;
  }
  uint8_t spcr;
  uint8_t spsr;
  friend class SPIClass;
};
*/

class SPIClass {
public:
  // Initialize the SPI library
  static void begin();

  // If SPI is used from within an interrupt, this function registers
  // that interrupt with the SPI library, so beginTransaction() can
  // prevent conflicts.  The input interruptNumber is the number used
  // with attachInterrupt.  If SPI is used from a different interrupt
  // (eg, a timer), interruptNumber should be 255.
  static void usingInterrupt(uint8_t interruptNumber);
  // And this does the opposite.
  static void notUsingInterrupt(uint8_t interruptNumber);
  // Note: the usingInterrupt and notUsingInterrupt functions should
  // not to be called from ISR context or inside a transaction.
  // For details see:
  // https://github.com/arduino/Arduino/pull/2381
  // https://github.com/arduino/Arduino/pull/2449

  // Before using SPI.transfer() or asserting chip select pins,
  // this function is used to gain exclusive access to the SPI bus
  // and configure the correct settings.
  inline static void beginTransaction(){//SPISettings settings) {
    /*if (interruptMode > 0) {
			//load settings
      //uint8_t sreg = SREG;
			//disable interrupts
      //noInterrupts();

      #ifdef SPI_AVR_EIMSK
      if (interruptMode == 1) {
        interruptSave = SPI_AVR_EIMSK;
        SPI_AVR_EIMSK &= ~interruptMask;
        SREG = sreg;
      } else
      #endif
      {
        interruptSave = sreg;
      }
    }*/

    #ifdef SPI_TRANSACTION_MISMATCH_LED
    if (inTransactionFlag) {
      pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
      digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
    }
    inTransactionFlag = 1;
    #endif

    //SPCR = settings.spcr;
    //SPSR = settings.spsr;
  }

  // Write to the SPI bus (MOSI pin) and also receive (MISO pin)
  inline static uint8_t transfer(uint8_t data) {
    
    /*
     * The following NOP introduces a small delay that can prevent the wait
     * loop form iterating when running at the maximum speed. This gives
     * about 10% more speed, even if it seems counter-intuitive. At lower
     * speeds it is unnoticed.
     */
    //asm volatile("nop");
		
		//SPI data register, Address offset: 0x0C
		//output data
		__IO uint8_t* address = (uint8_t*)&SPI1->DR;
		*address = data;
		// wait for TXE (Transmit buffer empty)
    while ((SPI1->SR & SPI_SR_BSY)!= 0) ; // wait
		
		// wait for RXNE (Receive buffer not empty)
    while ((SPI1->SR & SPI_SR_RXNE)!= SPI_SR_RXNE) ; // wait
		
		//for(int i = 0; i < 1000; i++);
		
		return SPI1->DR;
  }
	/*
  inline static uint16_t transfer16(uint16_t data) {
    union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } in, out;
    in.val = data;
    if (!(SPCR & _BV(DORD))) {
      SPI1->DR = in.msb;
      asm volatile("nop"); // See transfer(uint8_t) function
      while (!(SPSR & _BV(SPIF))) ;
      out.msb = SPI1->DR;
      SPI1->DR = in.lsb;
      asm volatile("nop");
      while (!(SPSR & _BV(SPIF))) ;
      out.lsb = SPI1->DR;
    } else {
      SPI1->DR = in.lsb;
      asm volatile("nop");
      while (!(SPSR & _BV(SPIF))) ;
      out.lsb = SPI1->DR;
      SPI1->DR = in.msb;
      asm volatile("nop");
      while (!(SPSR & _BV(SPIF))) ;
      out.msb = SPI1->DR;
    }
    return out.val;
  } */
  inline static void transfer(void *buf, int count) {
    if (count == 0) return;
    uint8_t *p = (uint8_t *)buf;
		// wait for TXE (Transmit buffer empty)
    while ((SPI1->SR & SPI_SR_TXE)!= SPI_SR_TXE) ; // wait
		//send first byte
    SPI1->DR = *p;
    while (--count > 0) {
      uint8_t out = *(p + 1);
      // wait for RXNE (Receive buffer not empty)
			while ((SPI1->SR & SPI_SR_RXNE)!= SPI_SR_RXNE) ; // wait
			//receive byte
      uint8_t in = SPI1->DR;
			// wait for TXE (Transmit buffer empty)
			while ((SPI1->SR & SPI_SR_TXE)!= SPI_SR_TXE) ; // wait
			//send next byte
      SPI1->DR = out;
      *p++ = in;
    }
    // wait for RXNE (Receive buffer not empty)
		while ((SPI1->SR & SPI_SR_RXNE)!= SPI_SR_RXNE) ; // wait
		//receive last byte
    *p = SPI1->DR;
  }
  // After performing a group of transfers and releasing the chip select
  // signal, this function allows others to access the SPI bus
  inline static void endTransaction(void) {
    #ifdef SPI_TRANSACTION_MISMATCH_LED
    if (!inTransactionFlag) {
      pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
      digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
    }
    inTransactionFlag = 0;
    #endif

   /* if (interruptMode > 0) {
      #ifdef SPI_AVR_EIMSK
      uint8_t sreg = SREG;
      #endif
      noInterrupts();
      #ifdef SPI_AVR_EIMSK
      if (interruptMode == 1) {
        SPI_AVR_EIMSK = interruptSave;
        SREG = sreg;
      } else
      #endif
      {
        SREG = interruptSave;
      }
    }*/
  }

  // Disable the SPI bus
  static void end();
/*
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  inline static void setBitOrder(uint8_t bitOrder) {
    if (bitOrder == LSBFIRST) SPCR |= _BV(DORD);
    else SPCR &= ~(_BV(DORD));
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  inline static void setDataMode(uint8_t dataMode) {
    SPCR = (SPCR & ~SPI_MODE_MASK) | dataMode;
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  inline static void setClockDivider(uint8_t clockDiv) {
    SPCR = (SPCR & ~SPI_CLOCK_MASK) | (clockDiv & SPI_CLOCK_MASK);
    SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((clockDiv >> 2) & SPI_2XCLOCK_MASK);
  }
  // These undocumented functions should not be used.  SPI.transfer()
  // polls the hardware flag which is automatically cleared as the
  // AVR responds to SPI's interrupt
  inline static void attachInterrupt() { SPCR |= _BV(SPIE); }
  inline static void detachInterrupt() { SPCR &= ~_BV(SPIE); }
*/
private:
  static uint8_t initialized;
  static uint8_t interruptMode; // 0=none, 1=mask, 2=global
  static uint8_t interruptMask; // which interrupts to mask
  static uint8_t interruptSave; // temp storage, to restore state
  #ifdef SPI_TRANSACTION_MISMATCH_LED
  static uint8_t inTransactionFlag;
  #endif
};

extern SPIClass SPI;

#endif
