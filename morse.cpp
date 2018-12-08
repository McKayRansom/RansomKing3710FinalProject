
#include "morse.h"
#include "stm32l476xx.h"
#include "SPI.h"
#include "USART.h"
letter  five  ={'5', 0, 0};
letter  four  ={'4', 0 , 0};
letter  three  ={'3', 0 , 0};
letter  two  ={'2', 0 , 0};
letter  one  ={'1', 0 , 0};
letter  six  ={'6' , 0 , 0};
letter  seven  ={'7' , 0 , 0};
letter  eight  ={'8' , 0 , 0};
letter  nine  ={'9' , 0 , 0};
letter  zero  ={'0' , 0 , 0};
letter  h  ={'h', &five, &four};
letter  v  ={'v', 0, &three};
letter  f  ={'f'  ,   0 , 0};
letter  hyphen  ={'-' ,   0, &two};
letter  l  ={'l', 0 , 0};
letter  p  ={'p', 0 , 0};
letter  j  ={'j', 0 , &one};
letter  b  ={'b', &six, 0};
letter  x  ={'x', 0 , 0};
letter  c  ={'c', 0 , 0};
letter  y  ={'y', 0 , 0};
letter  z  ={'z', &seven, 0};
letter  q  ={'q', 0 , 0};
letter  period ={'.', &eight, 0};
letter  nothing ={'~' ,&nine ,&zero};
letter  s  ={'s', &h ,  &v};
letter  u  ={'u', &f ,  &hyphen};
letter  r  ={'r', &l ,   0};
letter  w  ={'w', &p ,  &j};
letter  d  ={'d', &b ,  &x};
letter  k  ={'k', &c ,  &y};
letter  g  ={'g', &z ,  &q};
letter  o  ={'o', &period, &nothing};
letter  i  ={'i', &s , &u};
letter  a  ={'a', &r , &w};
letter  n  ={'n', &d , &k};
letter m ={'m',  &g ,  &o};
letter  e  ={'e',  &i ,  &a};
letter  t  ={'t',  &n ,  &m};
letter  root  ={0 , &e , &t};

// This  pointer  w i l l  traverse  the  tree .
letter* toLetter =  &root;


void buzzz(int durration, int speed)
{
	GPIOB->MODER &= ~GPIO_MODER_MODE2;
	GPIOB->MODER |= GPIO_MODER_MODE2_0;
	for(int i = 0; i < durration; i++)
	{
		GPIOB->ODR ^= 1u << 2;
		wait(speed);
	}
}

const int munit = 200; 

void dit()
{
	buzzz(munit,400);
	wait(400*1*munit);
}

void dah()
{
	buzzz(3*munit,400);
	wait(400*1*munit);
}

void newMChar()
{
	wait(400*2*munit);
}

void space()
{
	wait(4*munit*400);
}
void morseOut(char letter)
{
	if(letter == ' ')
		space();
	
	if(letter >= 'a' && letter <= 'z')
		letter -= 0x20;
	
	if(letter <= 'Z' && letter >= 'A')
	{
	switch(letter)
	{
		case 'A':
			dit();
			dah();
		break;

		case 'B':
			dah();
			dit();
			dit();
			dit();
		break;
		
		case 'C':
			dah();
			dit();
			dah();
			dit();
		break;	
		
		case 'D':
			dah();
			dit();
			dit();
		break;
		
		case 'E':
			dit();
		break;
		
		case 'F':
		dit();
		dit();
		dah();
		dit();
		break;
		
		case 'G':
		dah();
		dah();
		dit();
		break;
		
		case 'H':
		dit();
		dit();
		dit();
		dit();
		break;
		
		case 'I':
		dit();
		dit();
		break;
		
		case 'J':
		dit();
		dah();
		dah();
		dah();
		break;
		
		case 'K':
		dah();
		dit();
		dah();
		break;
		
		case 'L':
		dit();
		dah();
		dit();
		dit();
		break;
		
		case 'M':
		dah();
		dah();
		break;
		
		case 'N':
		dah();
		dit();
		break;
		
		case 'O':
		dah();
		dah();
		dah();
		break;
		
		case 'P':
		dit();
		dah();
		dah();
		dit();
		break;
		
		case 'Q':
		dah();
		dah();
		dit();
		dah();
		break;
		
		case 'R':
		dit();
		dah();
		dit();
		break;
		
		case 'S':
		dit();
		dit();
		dit();
		break;
		
		case 'T':
		dah();
		break;
		
		case 'U':
		dit();
		dit();
		dah();
		break;
		
		case 'V':
		dit();
		dit();
		dit();
		dah();
		break;
		
		case 'W':
		dit();
		dah();
		dah();
		break;
		
		case 'X':
		dah();
		dit();
		dit();
		dah();
		break;
		
		case 'Y':
		dah();
		dit();
		dah();
		dah();
		break;
		
		case 'Z':
		dah();
		dah();
		dit();
		dit();
		break;
	}
	}
			
}
int lastJoySendValue = 0;
int lastJoyValue = 0;
int lastJoyTime = 0;
//returns character converted or 0
char morseIn() {
	
	//joy Down = PA5
	int currentJoySendValue = !!(GPIOA->IDR & (1U << 5));
	if ((currentJoySendValue == 1) && (lastJoySendValue == 0)) {
		return '\r';
	}
	lastJoySendValue = currentJoySendValue;
	//joystick up is PA3 
	char retVal = 0;
	int currentJoyValue = !!(GPIOA->IDR & (1U << 3));
	int currentTime = millis();
	int elapsedTime = currentTime - lastJoyTime;
	if (elapsedTime > 150) {
		retVal = toLetter->letter;
		toLetter = &root;
		//Serial.println("TIME");
		lastJoyTime = currentTime;
	}
	if (currentJoyValue != lastJoyValue) {
		
		
		//if the joystick was just pressed
		//this means the previous state was wait
		if (currentJoyValue) {
		//longest wait time between letters
			if (elapsedTime > 150) {
				//assume a full char has been input
				retVal = toLetter->letter;
				toLetter = &root;
				//Serial.println("space");
			}
		}
		//joystick was pressed down for dot durration
		else if (elapsedTime < 50 && elapsedTime > 10) {
			if (toLetter->left != 0) {
				toLetter = toLetter->left;
				Serial.println("DOT");
			}
		}
		//joystick was pressed down for dash durration
		else if (elapsedTime > 50) {
			if (toLetter->right != 0) {
				toLetter = toLetter->right;
				Serial.println("DASH");
			}
		}
		
		lastJoyValue = currentJoyValue;
		lastJoyTime = currentTime;
	}
	return retVal;
}
		
	
void wait (int ticks) {
	int i = 0;
	while (true) {
		i++;
		if (i >= ticks) {
			break;
		}
	}
}


