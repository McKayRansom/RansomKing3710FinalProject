//#include "main.h"
#include "stm32l476xx.h"
#include <string.h>

#include "LCD.h"
//#include "SPI.h"
#include "RFM69.h"
#include "USART.h"
#define false 0
//#define maybe 0.5
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
// CS43L22 Audio DAC Stereo (I2C address 0x94):  
//   SAI1_MCK = PE2     SAI1_SD  = PE6    I2C1_SDA = PB7    Audio_RST = PE3    
//   SAI1_SCK = PE5     SAI1_FS  = PE4    I2C1_SCL = PB6                                           
//
// MP34DT01 Digital MEMS microphone 
//    Audio_CLK = PE9   Audio_DIN = PE7
//
// LSM303C eCompass (a 3D accelerometer and 3D magnetometer module): 
//   MEMS_SCK  = PD1    MAG_DRDY = PC2    XL_CS  = PE0             
//   MEMS_MOSI = PD4    MAG_CS  = PC0     XL_INT = PE1       
//                      MAG_INT = PC1 
//
// L3GD20 Gyro (three-axis digital output): 
//   MEMS_SCK  = PD1    GYRO_CS   = PD7
//   MEMS_MOSI = PD4    GYRO_INT1 = PD2
//   MEMS_MISO = PD3    GYRO_INT2 = PB8
//
// ST-Link V2 (Virtual com port, Mass Storage, Debug port): 
//   USART_TX = PD5     SWCLK = PA14      MFX_USART3_TX   MCO
//   USART_RX = PD6     SWDIO = PA13      MFX_USART3_RX   NRST
//   PB3 = 3V3_REG_ON   SWO = PB5      
//
// Quad SPI Flash Memory (128 Mbit)
//   QSPI_CS  = PE11    QSPI_D0 = PE12    QSPI_D2 = PE14
//   QSPI_CLK = PE10    QSPI_D1 = PE13    QSPI_D3 = PE15
//
// LCD (24 segments, 4 commons, multiplexed 1/4 duty, 1/3 bias) on DIP28 connector
//   VLCD = PC3
//   COM0 = PA8     COM1  = PA9      COM2  = PA10    COM3  = PB9
//   SEG0 = PA7     SEG6  = PD11     SEG12 = PB5     SEG18 = PD8
//   SEG1 = PC5     SEG7  = PD13     SEG13 = PC8     SEG19 = PB14
//   SEG2 = PB1     SEG8  = PD15     SEG14 = PC6     SEG20 = PB12
//   SEG3 = PB13    SEG9  = PC7      SEG15 = PD14    SEG21 = PB0
//   SEG4 = PB15    SEG10 = PA15     SEG16 = PD12    SEG22 = PC4
//   SEG5 = PD9     SEG11 = PB4      SEG17 = PD10    SEG23 = PA6
// 
// USB OTG
//   OTG_FS_PowerSwitchOn = PC9    OTG_FS_VBUS = PC11    OTG_FS_DM = PA11  
//   OTG_FS_OverCurrent   = PC10   OTG_FS_ID   = PC12    OTG_FS_DP = PA12  
//
// PC14 = OSC32_IN      PC15 = OSC32_OUT
// PH0  = OSC_IN        PH1  = OSC_OUT 
// 
// PA4  = DAC1_OUT1 (NLMFX0 WAKEUP)   PA5 = DAC1_OUT2 (Joy Down)
// PA3  = OPAMP1_VOUT (Joy Up)        PB0 = OPAMP2_VOUT (LCD SEG21)
//
//****************************************************************************************************************
void SysTick_Initialize (uint32_t ticks);
void Keypad_initialize(void);
void pinMode(GPIO_TypeDef* GPIOx, int pin, int mode);
void pinAlternateFunction(GPIO_TypeDef* GPIOx, int pin, int mode);
void digitalWrite(GPIO_TypeDef* GPIOx, int pin, int value);
void wait (int ticks);
void morse(char letter);
void buzzz(int a, int b);
	
void test() {
	// Initializing USART
	testUART();
}

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

const int munit = 100; 

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

	
//slave select pin, interrupt pin
RFM69 radio(12, 10);

extern "C" void EXTI15_10_IRQHandler() {
	if ((EXTI->PR1 & EXTI_PR1_PIF10) == EXTI_PR1_PIF10) {
		//call radio interrupt handler
		radio.isr0();
		
		//Clear pending interrupt flag
		EXTI->PR1 |= EXTI_PR1_PIF10;
	}
		
}




//******* MAIN ******* //
int main(void)
{
	char testString[] = "SOS";
	//Switch system clock to HSI here
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	
	//LCD initialization and clearing the screen
	LCD_Initialization();
	LCD_Clear();
	
	//Display start up message
	
	
	//Initialize Keypad
	//Keypad_initialize();
	
	test();

	
	SysTick_Initialize(16000);
	
	for(int i = 0; i < strlen(testString); i++)
	{
//		morse(testString[i]);
//		newMChar();
	}
	

	
	
	LCD_DisplayWelcome();
	wait(1000000);
	LCD_Clear();
	LCD_DisplayString((uint8_t*)testString);
	
	
	
	
	if (!radio.initialize(FREQUENCY, MYNODEID, NETWORKID)) {
		Serial.println("FAILED RADIO INIT!");
		while(1);
	}
	Serial.print("Node: ");
	Serial.print(MYNODEID, DEC);
	Serial.println("finished radio init \n\r");
	radio.readAllRegs();
	
	
	uint32_t buffSize = 0;
	char buffer[36];
	char message[] = "TEST";
	if (radio.sendWithRetry(TONODEID, message, 4))
			Serial.println("ACK received!");
	else
		Serial.println("no ACK received :(");
	while (1) 
	{
		if (Serial.available()) {
			char inChar = Serial.read();
			buffer[buffSize++] = inChar;
			if (buffSize > 10 || (inChar == '\r' || inChar == '\n')) {
				radio.send(TONODEID, buffer, buffSize);
				buffSize = 0;
			}
		//Serial.Read(message, 1);
		//for(int i = 0; i < buffSize; i++)
		//{
			//LCD_DisplayString((uint8_t*) message);
			//buffer[buffSize++] = message[0];		
		}
		if (radio.receiveDone()) \
		{
			for (int i = 0; i < radio.DATALEN; i++)
			{
				Serial.write((char)radio.DATA[i]);
				
			}
			//Serial.Write((uint8_t*)radio.DATA, radio.DATALEN);
			
			LCD_Clear();
			LCD_DisplayString((uint8_t*)radio.DATA);
				
			
			
			for(int i = 0; i < radio.DATALEN; i++)
			{
				morse((char)radio.DATA[i]);
			}
			space();
			
			
			if (radio.ACKRequested()) {
				radio.sendACK();
				Serial.println("ACK SENT!");
			}
		}
		
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

void wait (int ticks) {
	int i = 0;
	while (true) {
		i++;
		if (i >= ticks) {
			break;
		}
	}
}




void morse(char letter)
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


void Keypad_initialize(void){
	//enable clock to gpios
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	//configure input (0b00)
	GPIOA->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE5);
	//configure output (0b01)
	GPIOE->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12 | GPIO_MODER_MODE13);
	GPIOE->MODER |= GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0 | GPIO_MODER_MODE12_0 | GPIO_MODER_MODE13_0;
	GPIOE->OTYPER |= GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13;
}

unsigned char Keypad_map[4][4] = {
	{'1','2','3','A'},
	{'4','5','6','B'},
	{'7','8','9','C'},
	{'*','0','#','D'},
};

unsigned char Keypad_Scan(void)
{
//	int count = 0;
	unsigned char row,col, ColumnPressed;
	unsigned char key = 0xFF;

	unsigned int cols[4] = {44,42,38,14};
	//unsigned int rows[4] = {14336, 13312, 11264, 7168};

	GPIOE->ODR &= (0xFFFFC3FF); //set rows to zero
	wait(4000); // wait

	if((GPIOA->IDR & 0x0000002E) == 0x0000002E){
		return 0xFF;
	}

	for(col =0; col<4; col++){
		if((GPIOA->IDR & 0x0000002E) == cols[col]){
			ColumnPressed = col;
			break;
		}
	}
	for(row =0; row<4; row++){
		GPIOE->ODR |= GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12 | GPIO_ODR_OD13;
		GPIOE->ODR &= ~(GPIO_ODR_OD10 << row);
		wait(4000); // wait
		if((GPIOA->IDR & 0x0000002E) != 0x02E){
			//while((GPIOA->IDR & 0x0000002E) == cols[ColumnPressed]);
			key = Keypad_map[row][ColumnPressed];
			break;
		}
	}
	return key;
}


void Joystick_Initialize() {
	// Enable GPIO Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// GPIO Mode: Input(00), Output (01),
	// AF(10), Analog (11)
	GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2);
	// GPIO Push-Pull: No pull-up, pull-down (00),
	// Pull-up (01), Pull-down (10), Reserved (11)
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1 | GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1; // Pull down
	NVIC_EnableIRQ(EXTI0_IRQn); // Enable Interrupt
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
}
