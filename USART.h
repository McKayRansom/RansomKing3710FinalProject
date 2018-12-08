#ifndef USART_H
#define USART_H
#include <string>
#include "stm32l476xx.h"
using std::string;

#define DEC 0
#define BIN 1
#define HEX 2
class USART {
	public:
		USART();
		USART(USART_TypeDef* USARTn);
		void prep();
		void Init();
		void read(char* buffer, int nBytes);
		char read();
		bool available();
		void write(char chr);
		void write(const char* buffer, int nBytes);
		void print(string str);
		void print(int value, int mode);
		void println(int value, int mode);
		void println(std::string str);
		void println();
	private:
		USART_TypeDef* USARTx;
};
//#pragma once
extern USART Serial;
extern "C" void Serial_Init();
#endif
