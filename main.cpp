/*****************************************************
*	ECE 3710 Final Project
*		radio communication
*	McKay Ransom
*	Glendyn King
*	Fall 2018
*
*
* Psudo code:
*
* Initialize(everything)
* print(debug info)
* send(test message)
* infinite loop:
* 	recevie messages
* 		print recevied messages
*			beep recevied messages
*		serial input
*			send serial input
*		joystick input
*			send joystick input
*****************************************************/
#include "stm32l476xx.h"
#include <string.h>
#include "morse.h"
#include "LCD.h"
#include "RFM69.h"
#include "USART.h"
#define false 0
#define true 1
#define NETWORKID 19
#define MYNODEID 0
#define TONODEID 0
#define FREQUENCY RF69_915MHZ

//*************************************  32L476GDISCOVERY ***************************************************************************
// STM32L4:  STM32L476VGT6 MCU = ARM Cortex-M4 + FPU + DSP, 
//           LQFP100, 1 MB of Flash, 128 KB of SRAM
//           Instruction cache = 32 lines of 4x64 bits (1KB)
//           Data cache = 8 lines of 4x64 bits (256 B)
//
// Joystick (MT-008A): 
//   Right = PA2        Up   = PA3         Center = PA0
//   Left  = PA1        Down = PA5
//
// User LEDs: 
//   LD4 Red   = PB2    LD5 Green = PE8
//   
// ST-Link V2 (Virtual com port, Mass Storage, Debug port): 
//   USART_TX = PD5     SWCLK = PA14      MFX_USART3_TX   MCO
//   USART_RX = PD6     SWDIO = PA13      MFX_USART3_RX   NRST
//   PB3 = 3V3_REG_ON   SWO = PB5      
//
//****************************************************************************************************************
void SysTick_Initialize (uint32_t ticks);
void Keypad_initialize(void);
void pinMode(GPIO_TypeDef* GPIOx, int pin, int mode);
void pinAlternateFunction(GPIO_TypeDef* GPIOx, int pin, int mode);
void digitalWrite(GPIO_TypeDef* GPIOx, int pin, int value);
void Joystick_Initialize();
	
//slave select pin, interrupt pin
RFM69 radio(12, 10);






//******* MAIN ******* //
int main(void)
{
	//Switch system clock to HSI here
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	
	//LCD initialization and clearing the screen
	LCD_Initialization();
	LCD_Clear();
	
	//Serial init function (in USART.cpp)
	Serial_Init();
	
	//initialize Systick interrupt
	SysTick_Initialize(16000);
	
	//LCD displays test string
	LCD_DisplayWelcome();
	
	//Initialize Joystick
	Joystick_Initialize();
	
	// Initilize Radio section
	if (!radio.initialize(FREQUENCY, MYNODEID, NETWORKID)) {
		Serial.println("FAILED RADIO INIT!");
		//dead loop because we can't continue
		while(1);
	}
	radio.setHighPower();
	radio.readAllRegsCompact();
	Serial.print("Node: ");
	Serial.print(MYNODEID, DEC);
	Serial.println("finished radio init \n\r");
	
	
	uint32_t buffSize = 0;
	//max send size on radio is 61


	char buffer[RF69_MAX_DATA_LEN];
	char message[] = "TEST";
	if (radio.sendWithRetry(TONODEID, message, 4))
			Serial.println("ACK received!");
	else
		Serial.println("no ACK received :(");
	
	//Infinite loop where usefull work is done
	while (1) 
	{
		if (Serial.available()) {
			char inChar = Serial.read();
			buffer[buffSize++] = inChar;
			if (buffSize > RF69_MAX_DATA_LEN || (inChar == '\r' || inChar == '\n')) {
				if (!radio.sendWithRetry(TONODEID, buffer, buffSize))
					Serial.println("NO ACK RECEIVED!");
				buffSize = 0;
			}
		}
		if (radio.receiveDone()) \
		{
			int dataLength = radio.DATALEN;
			char data[RF69_MAX_DATA_LEN];
			for (int i = 0; i < radio.DATALEN; i++)
				data[i] = (char)radio.DATA[i];
			
			//send acknowledge first so it gets back in time!
			if (radio.ACKRequested())
				radio.sendACK();
				
			//write data over serial
			for (int i = 0; i < dataLength; i++)
				Serial.write(data[i]);
			
			//display string on LCD (as much as will fit anyway)
			LCD_Clear();
			LCD_DisplayString((uint8_t*) data);
				
			//beep out data in morse code
			for(int i = 0; i < dataLength; i++)
				morseOut(data[i]);
			
			space();
		}
		char inChar = morseIn();
		if (inChar != 0) 
		{
			LCD_WriteChar((uint8_t*)&inChar, false, false, 0);
			Serial.write(inChar);
			buffer[buffSize++] = inChar;
			if (buffSize > RF69_MAX_DATA_LEN || (inChar == '\r' || inChar == '\n')) {
				if (!radio.sendWithRetry(TONODEID, buffer, buffSize))
					Serial.println("NO ACK RECEIVED!");
				buffSize = 0;
			}
		}
		
	}
}

extern "C" void EXTI15_10_IRQHandler() {
	if ((EXTI->PR1 & EXTI_PR1_PIF10) == EXTI_PR1_PIF10) {
		//call radio interrupt handler
		radio.isr0();
		
		//Clear pending interrupt flag
		EXTI->PR1 |= EXTI_PR1_PIF10;
	}
}



//******* END OF MAIN ******* //
int counter = 0;
extern "C" void SysTick_Handler (void) { // SysTick interrupt service routine
	counter++;
} 

int millis() {
	return counter;
}


void SysTick_Initialize (uint32_t ticks) {
	SysTick->CTRL = 0; // Disable SysTick
	SysTick->LOAD = ticks - 1; // Set reload register
	// Set interrupt priority of SysTick to least urgency (i.e., largest priority value)
	NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
	SysTick->VAL = 0; // Reset the SysTick counter value
	// Select processor clock: 1 = processor clock; 0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	// Enables SysTick interrupt, 1 = Enable, 0 = Disable
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	// Enable SysTick
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

// 00 = input, 01 = Output, 10 = Alternate Function, 11 =  Analog
void pinMode(GPIO_TypeDef* GPIOx, int pin, int mode) {
	GPIOx->MODER &= ~(3U << (2*pin));
	GPIOx->MODER |= mode << (2*pin);
}

void pinAlternateFunction(GPIO_TypeDef* GPIOx, int pin, int mode) {
	if (pin < 8) {
		GPIOx->AFR[0] &= ~(0xF << (4*pin));
		GPIOx->AFR[0] |= mode << (4*pin);
	} else {
		GPIOx->AFR[1] &= ~(0xF << (4*(pin - 8)));
		GPIOx->AFR[1] |= mode << (4*(pin - 8));
	}
}

void digitalWrite(GPIO_TypeDef* GPIOx, int pin, int value) {
	if (value) {
		GPIOx->ODR |= 1 << pin;
	} else {
		GPIOx->ODR &= ~(1 << pin);
	}
}

int digitalRead(GPIO_TypeDef* GPIOx, int pin) {
	return !!(GPIOx->IDR & (1U << pin));
}




void Joystick_Initialize() {
	// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// GPIO Mode: Input(00), Output (01),
	// AF(10), Analog (11)
	GPIOA->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE5);// | GPIO_MODER_MODE2);
	// GPIO Push-Pull: No pull-up, pull-down (00),
	// Pull-up (01), Pull-down (10), Reserved (11)
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD5);// | GPIO_PUPDR_PUPD2);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD3_1 | GPIO_PUPDR_PUPD5_1;// | GPIO_PUPDR_PUPD2_1; // Pull down
	/*NVIC_EnableIRQ(EXTI0_IRQn); // Enable Interrupt
	NVIC_EnableIRQ(EXTI1_IRQn); // Enable Interrupt
	NVIC_EnableIRQ(EXTI2_IRQn); // Enable Interrupt
	NVIC_SetPriority(EXTI0_IRQn, 0);
	NVIC_SetPriority(EXTI1_IRQn, 0);
	NVIC_SetPriority(EXTI2_IRQn, 0); 
	// Connect External Line to the GPI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI2 | SYSCFG_EXTICR1_EXTI1 | SYSCFG_EXTICR1_EXTI0);
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA | SYSCFG_EXTICR1_EXTI1_PA | SYSCFG_EXTICR1_EXTI0_PA;
	// Interrupt Mask Register
	// 0 = marked, 1 = not masked (enabled)
	EXTI->IMR1 |= EXTI_IMR1_IM2 | EXTI_IMR1_IM1 | EXTI_IMR1_IM0;
	// Rising trigger selection
	// 0 = trigger disabled, 1 = trigger enabled
	EXTI->RTSR1 |= EXTI_RTSR1_RT2 | EXTI_RTSR1_RT1 | EXTI_RTSR1_RT0; 
	*/
}
