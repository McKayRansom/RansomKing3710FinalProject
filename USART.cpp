
#include "USART.h"
//#include <stdio.h>
//#include <string>
USART Serial(USART2);
using std::string;
//using std::itoa;

extern "C" void Serial_Init() {
	Serial.prep();
	Serial.Init();
	Serial.print("Startup!\n");
}
USART::USART()
{
	USARTx = USART2;
}

USART::USART(USART_TypeDef* USARTn) {
	USARTx = USARTn;
}

void USART::prep(void)
{
	//Enable GPIO clock and configure the tx pin and the rx pin as:
	//alternate function, highspeed, push-pull, pullup
	
	//--------------- GPIO initialization for USART 2 ------------
	// PD.5 = AF7 (USART1_TX), PD.6 = AF7 (USART2_RX), get appendix removed
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN; // enable GPIO port B clock
	
	// 00 = input, 01 = Output, 10 = Alternate Function, 11 =  Analog
	GPIOD->MODER &= ~(0xF<<(2*5)); // Clear mode bits for pin 5 and 6
	GPIOD->MODER |= (0xA<<(2*5)); // set AF for pin 5 and 6
	
	// Alternative function 7 = USART 2
	GPIOD->AFR[0] |= 0x77 << (4*5); // set pin 5 and 7 to AF 7
	
	// GPIO Speed 00 = low speed, 01 = medium speed
	//						10 = Fast speed, 11 = high speed
	GPIOD->OSPEEDR |= 0xF<<(2*5);
	
	//GPIO Push-pull select pull-up
	GPIOD->PUPDR &= ~(0xF<<(2*5));
	GPIOD->PUPDR |= (0x5<<(2*5));
	
	// GPIO Output Type: 0 = push-pull, 1 = open drain
	GPIOD->OTYPER &= ~(0x3<<5);
	
	//nm//----------------GPIO Initialization for USART 
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	
	// Select system clock (SYSCLK) USART clock source of UART 2
	// 00 = PCLK,  01 = SYSCLK
	//10 = HSI16, 11 = LSE
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);
	RCC->CCIPR |= (RCC_CCIPR_USART2SEL_1);
}


//USART initialize, 
//takes usart pointer
//returns void
void USART::Init()
{
	// Disable USART
	USART2->CR1 &= ~USART_CR1_UE;
	
	// Set data Length to 8 bits
	// 00 = 8 data bits, 01 = 9 data bits, 10 = 7 data bits
	USART2->CR1 &= ~USART_CR1_M;
	
	// Select 1 stop bit
	// 00 = 1 stop bit 01 - 0.5 stop bit
	// 10 = 2 stop bits 11  = 1.5 stop bits
	USART2->CR2 &= ~USART_CR2_STOP;
	
	// Oversampling by 16
	//0 = oversampling by 16, 1 = oversampling by 8 
	USART2->CR1&=~USART_CR1_OVER8;
	
	
	//set baud
	USART2->BRR = 0x683;
	
	// Enable transmission and reception
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
	
	//Enable transmission and reception
	USART2->CR1 |= USART_CR1_UE;
	
	//Verify that usart is ready for transmission
	//teack: transmit enable acknoledge flag. Hardware sets or resets it.
	while((USART2->ISR & USART_ISR_TEACK) == 0);
	
	// Verify that USART is ready for reception
	//REACK: receive enable acknowledge flag. Hardware sets or resets it.
	while((USART2->ISR & USART_ISR_REACK) == 0);
}

bool USART::available() {
	return (USARTx->ISR & USART_ISR_RXNE);
}

void USART::read(char* buffer, int nBytes)
{	
	for(int i = 0; i < nBytes; i++)
	{
		while(!(USARTx->ISR & USART_ISR_RXNE));
		buffer[i] = USARTx->RDR;
	}
}

char USART::read()
{	
	while(!(USARTx->ISR & USART_ISR_RXNE));
	return USARTx->RDR;
}

void USART::write(char byte)
{	
	while(!(USARTx->ISR & USART_ISR_TXE));
	USARTx->TDR = byte;
	
	while(!(USARTx->ISR & USART_ISR_TC));
	
	USARTx->ICR |= USART_ICR_TCCF;
}

void USART::write(const char* buffer, int nBytes)
{	
	for(int i = 0; i < nBytes; i++)
	{
		while(!(USARTx->ISR & USART_ISR_TXE));
		USARTx->TDR = buffer[i] & 0xFF;
	}
	
	while(!(USARTx->ISR & USART_ISR_TC));
	
	USARTx->ICR |= USART_ICR_TCCF;
}

void USART::print(string str) {
	write(str.c_str(), str.size());
}

char intToChar(int num) {
	switch(num) {
		case 0:
			return '0';
		case 1:
			return '1';
		case 2:
			return '2';
		case 3:
			return '3';
		case 4:
			return '4';
		case 5:
			return '5';
		case 6:
			return '6';
		case 7:
			return '7';
		case 8:
			return '8';
		case 9:
			return '9';
	}
	return '@';
}

void USART::print(int value, int mode) {
	string str;
	if (value == 0)
		str = str + '0';
	while (value > 0) {
		str = str + intToChar(value % (10));
		value = value /10;
	}
	USART::print(str);
}
void USART::println(int value, int mode) {
	print(value, mode);
	println();
}
void USART::println(std::string str) {
	print(str + "\r");
}
void USART::println() {
	write("\r", 2);
}
