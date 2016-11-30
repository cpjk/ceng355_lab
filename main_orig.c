#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// ----- main() ---------------------------------------------------------------
// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#define DELAY 48000

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)

/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

//spi usefull functions
SPI_InitTypeDef SPI_InitStructInfo;
SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;

void myGPIO_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
void mySPI_Init(void);

void SPISendData(uint8_t);
void Delay(volatile long);
void LCD_Init(void);
void CommandSend(uint8_t);
void CommandEnable(uint8_t);
void SendToLCD(uint8_t);
void DataEnable(uint8_t);
void DisplayResistance(int);
void DisplayFrequency(int);

unsigned int count;
unsigned int edge = 0;
int freq = 0;
int thousand = 0;
int hundred = 0;
int ten = 0;
int one = 0;
int resis = 0;

int main(int argc, char* argv[])
{
	/*INITIALIZATIONS*/
	myGPIO_Init();            /* Initialize I/O port PA */
	myTIM2_Init();            /* Initialize timer TIM2 */
	myEXTI_Init();            /* Initialize EXTI */
	myADC_Init();            /* initialize ADC*/
	myDAC_Init();            /* initialize DAC*/
	mySPI_Init();            /* initialize SPI*/
	LCD_Init();              /* initialize LCD*/

	trace_printf("initialization complete\n");

	// pause after every command and data byte sent.
	// also pause between nibbles
	while (1) //loops forever
	{
		//trace_printf("Inside main loop.\n");
		ADC1->CR = ADC_CR_ADSTART;                                    //Starting ADC
		while((ADC1->ISR & ADC_ISR_EOC) == 0);            //Wait until the ADC has completed conversions
		//trace_printf("ADC completed conversion.\n");

		DAC->DHR12R1 = ADC1->DR;                  //output signal of ADC to the input of the DAC

		resis = (ADC1->DR)*5000/4095;          //Resistance calculation
		//sendr();
		DisplayFrequency(freq);             //sends freq to Frequency display function
		//trace_printf("Freq: %d. Res: %d\n", freq, resis);
		DisplayResistance(resis);            //sends resis to the resistance display function
		// trace_printf("\n");
	}

	return 0;
}

void DisplayResistance(int Resistance){
	thousand = Resistance/1000;                    //parsing the resistance data and breaking it up into categories
	hundred = (Resistance/100) % 10;
	ten = (Resistance/10) % 10;
	one = (Resistance) % 10;

	CommandSend(0xC0);            //setting display to be on the second line of the LCD

	SendToLCD(0x52);    //"R"
	SendToLCD(0x3A);          //":"
	SendToLCD(thousand+ '0');    //send each digit
	SendToLCD(hundred+ '0');    //+ '0' sends the digit to the correct ASCII number
	SendToLCD(ten+ '0');        // could use + 48 instead
	SendToLCD(one+ '0');

	SendToLCD(0xF4);    //"O"
}

void sendr() {
	CommandSend(0xC0);            //setting display to be on the second line of the LCD
	Delay(DELAY);
	SendToLCD(0x52);    //"R"
	Delay(DELAY);
}

void DisplayFrequency(int Frequency){
	thousand = Frequency/1000 % 10;            //Separating Frequency value into each digit
	hundred = (Frequency/100) % 10;
	ten = (Frequency/10) % 10;
	one = (Frequency) % 10;

	CommandSend(0x80);            //setting display to start with the LCD's first line

	SendToLCD(0x46);    //"F"
	SendToLCD(0x3A);    //":"

	SendToLCD(thousand+'0');    //sending each digit
	SendToLCD(hundred+'0');            //+ '0' sends the digit to the correct ASCII number
	SendToLCD(ten+'0');                    // could use + 48 instead
	SendToLCD(one+'0');

	SendToLCD(0x48);    //"H"
	SendToLCD(0x7A);    //"z"
}

void SPISendData(uint8_t Data)
{
	//Force LCK signal to 0
	GPIOB->BRR |= GPIO_BRR_BR_4;                    //set lock signal to 0 (freeze the ports)
	Delay(2500);                                    //GPIO delay

	while((SPI1->SR & SPI_SR_BSY) != 0);            //wait until SPI is ready
	SPI_SendData8(SPI1, Data);                      //send data to the SPI

	//trace_printf("%x\n", Data);

	//trace_printf()
	//Test buffer value
	//trace_printf("%u\n", SPI1->DR);

	while((SPI1->SR & SPI_SR_BSY) != 0);            //wait again until SPI is ready

	//Force LCK signal to 1
	GPIOB->BSRR = GPIO_BSRR_BS_4;                   //set lock signal to 1     (enable the port)
	Delay(2500);                                    //GPIO delay
}

void DataEnable(uint8_t Word) // good
{
	uint8_t L = Word | 0x40;      //set high nibble to 0100  -> RS:1 -> LCD sees as DATA
	uint8_t H = Word | 0xC0;      //set high nibble to 1100  -> EN:1, RS:1 -> DATA, LCD READS

	SPISendData(L);        //send L to SPI ( send data)
	//Delay(700);            //SPI delay

	SPISendData(H);        //send H to SPI (send data and ask LCD to read it)
	//Delay(700);            //SPI delay

	SPISendData(L);        // (re-send data)
	// Delay(700);            //SPI delay
}

void CommandEnable(uint8_t Word) // good
{
	uint8_t EN = Word | 0x80;

	SPISendData(Word);            //send     the enable word to the SPI
	//Delay(700);                            //SPI delay

	SPISendData(EN);
	//Delay(700);                            //SPI delay

	SPISendData(Word);
	//Delay(700);                            //SPI delay
}

void CommandSend(uint8_t Word)
{
	uint8_t HighOrder = ((Word >> 4) & 0x0F);            //shifts the most significant bits to the least significant side and masks the most significant bits with 0
	CommandEnable(HighOrder);                                            //send shifted value to CommandEnable
	Delay(DELAY);
	uint8_t LowOrder = (Word & 0x0F);            //no need to shift bits, mask the higher order bits
	CommandEnable(LowOrder);                            //send values straight to CommandSend
	Delay(DELAY);
}

void Delay(volatile long delay)
{
	while (delay != 0)
	{
		delay--;
	}
}

void SendToLCD(uint8_t Word)
{
	//trace_printf("%c\n", Word);
	// cpjk: shift high nibble to low order bits clear the high order bits
	uint8_t HighOrder = ((Word >> 4) & 0x0F);   //shift msb to lsb and mask the msb with 0 (get rid of msb)
	DataEnable(HighOrder);                      //send the data
	Delay(DELAY);

	// cpjk: clear the high order bits so we can send just the low nibble
	uint8_t LowOrder = (Word & 0x0F);           //no need to shift, send lsb data
	DataEnable(LowOrder);
	Delay(DELAY);
}

void LCD_Init()
{
	/* Initial delay */
	Delay(DELAY);

	/*
    LCD may initially be in one of three states:
    (State1) 8-bit mode
    (State2) 4-bit mode, waiting for the first set of 4 bits
    (State3) 4-bit mode, waiting for the second set of 4 bits
	 */

	// Set D7-D4 to 0b0011, and toggle the enable bit.
	// this ensures the LCD is definitely in 8-bit mode
	for(int i = 0; i<3;i++)
	{
		SPISendData(0x03);
		//Delay(500000);
		//Delay(50000);

		SPISendData(0x83);
		//Delay(500000);
		//Delay(50000);

		SPISendData(0x03);
		// Delay(500000);
		//Delay(50000);
	}
	// LCD is in 8-bit mode

	// Toggle the enable bit while sending  (D7-D4) = 0010.
	// This sets the LCD in 4-bit mode
	SPISendData(0x02); // Send (D7-D4) - 0010 (RS=0)
	//Delay(500);
	SPISendData(0x82); // Send (D7-D4) - 1000 0010 (RS=0) (EN=1)
	//Delay(500);
	//SPISendData(0x82); // Send (D7-D4) - 0010 (RS=0)
	SPISendData(0x02); // Send (D7-D4) - 0010 (RS=0)
	//Delay(500);
	// LCD is now in 4-bit mode

	CommandSend(0x28); //DL = 0, N = 1, F = 0
	CommandSend(0x0C); //D = 1, C = 0, B = 0
	CommandSend(0x06); //I/D = 1, S = 0
	CommandSend(0x01); //Clears display

	Delay(DELAY);
}

void myGPIO_Init()
{
	/* Enable clock for GPIOA peripheral */
	//RCC->AHBENR = 0x10000;
	//relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;  //configure PA1 as input
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	//Configure PB1 as alternate function input
	GPIOB->AFR[2] = ((uint32_t)0x00000000);


	/* Configure PA1, PA2, PA4 to analog mode*/
	//relevant register: GPIOA->MODER
	GPIOA->MODER |= (GPIO_MODER_MODER2 | GPIO_MODER_MODER4);
	GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_1;

	/* Ensure no pull-up/pull-down for PA1 */
	//relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR2 | GPIO_PUPDR_PUPDR4);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5);
}

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;

	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(TIM2_IRQn,0);

	/* Enable TIM2 interrupts in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	TIM2->DIER |= TIM_DIER_UIE;
}

void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	SYSCFG->EXTICR[1] = 0x1000;

	/* EXTI1 line interrupts: set rising-edge trigger */
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn,0);

	/* Enable EXTI1 interrupts in NVIC */
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void myADC_Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;            //ADC1 clock enable
	ADC1->CFGR1 |= ADC_CFGR1_CONT;                    //this bit will ensure the ADC is in  continuous mosde and not single conversion mode
	ADC1->CHSELR |= ADC_CHSELR_CHSEL2;            //channel 2 is selected for conversion
	ADC1->SMPR |= ADC_SMPR_SMP;                            //sampling time selection (239.5 ADC cycles)
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);    //wait until ADC is ready
}

void myDAC_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;                    //DAC clock enable
	DAC->CR |= DAC_CR_EN1;                                //DAC channel 1 enable and set software trigger

	//Wake-up time
	DAC->CR |= DAC_CR_TSEL1;
	DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
}

void mySPI_Init()
{

	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI_InitStruct->SPI_Direction = SPI_Direction_1Line_Tx;            //setting the direction of the transmission line
	SPI_InitStruct->SPI_Mode = SPI_Mode_Master;                         //master mode
	SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;                    //size of data being sent
	SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;                           //idle clock polarity
	SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;                         //active clock edge is its first/second one
	SPI_InitStruct->SPI_NSS = SPI_NSS_Soft;                             //setting nss pin
	SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;    //rate of sending data to LCD
	SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;                    //set spi so that the data is in msb mode (first packet is msb, second packet is lsb)
	SPI_InitStruct->SPI_CRCPolynomial = 7;                                            //register contains polynomial for crc calculation
	SPI_Init(SPI1, SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	// Check if update interrupt flag is set
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n Overflow! \n");

		/* Clear update interrupt flag */
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		TIM2->CR1 &= ~(TIM_CR1_CEN);
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	count = TIM2->CNT;

	// Check if EXTI1 interrupt pending flag is set
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		if(edge == 0)    //If this is the first edge (rising)
		{
			TIM2->CNT = 0;                            // Clear count register (TIM2->CNT).
			TIM2->CR1 |= TIM_CR1_CEN;    // Start timer (TIM2->CR1).
			edge = 1;
		}
		else                     //Else this is the second edge (falling)
		{
			EXTI->IMR &= ~(EXTI_IMR_MR1);
			TIM2->CR1 &= ~(TIM_CR1_CEN);                    //Stop timer (TIM2->CR1).

			freq = SystemCoreClock/count;                    // Frequency in Hz
			edge = 0;
			EXTI->IMR |= EXTI_IMR_MR1;
		}

		EXTI->PR |= EXTI_PR_PR1; //Clear EXTI1 interrupt pending flag (EXTI->PR)
	}
}

#pragma GCC diagnostic pop
