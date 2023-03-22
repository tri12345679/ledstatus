/*
 * main.c
 *
 *  Created on: Feb 18, 2023
 *      Author: TRI
 */

/*
 * main.c
 *
 *  Created on: Feb 16, 2023
 *      Author: TRI
 */



#include<stdint.h>
void Clock_HSE_Init()
{
	uint32_t* CR =(uint32_t*)0x40023800;
	uint32_t* CFGR =(uint32_t*)0x40023808;
	*CR |= (1<<16); //enable HSE
	while (((*CR >>17)& 1) ==0); // wait HSE ready
	*CFGR |= (0b01 << 0); // set HSE as system clock
}
void Clock_PLL_Init()
{
	uint32_t* CR =(uint32_t*)0x40023800;
	uint32_t* CFGR =(uint32_t*)0x40023808;
	uint32_t* PLLCFGR = (uint32_t*)0x40023804;

	*CR |= (1<<24);
	while (((*CR >>25)& 1) ==0); // wait PLL ready

	// set HSI as source for PLL clock
	*PLLCFGR &= ~ (1<<22);
	// HSI (16Mzh -> M(/?) ? -> *N(*?) ->P(/?)100Mhz
	*PLLCFGR |= (16<<0) | (200<<6) | (0b00 <<16);

	*CFGR |= (0b10 << 0); // set PLL as system clock
}


#define RCC_ADDR_BASE 0x40023800
#define GPIOD_ADDR_BASE 0x40020c00


//#define GPIOD 0x40020C00

typedef struct
{
	char MODER0 : 2;
	char MODER1 : 2;
	char MODER2 : 2;
	char MODER3 : 2;
	char MODER4 : 2;
	char MODER5 : 2;
	char MODER6 : 2;
	char MODER7 : 2;
	char MODER8 : 2;
	char MODER9 : 2;
	char MODER10 : 2;
	char MODER11 : 2;
	char MODER12 : 2;
	char MODER13 : 2;
	char MODER14 : 2;
	char MODER15 : 2;
}MODER_t;
enum{
	INPUT,OUTPUT,ALTERNATE,ANALOG //,MODER12,MODER13,MODER14,MODER15
};

void LED_Init()
{


	uint32_t* RCC_AHB1ENR = (uint32_t*)0x40023830;
	*RCC_AHB1ENR |= (1 <<3);  // enable clock cho port D

//	uint32_t* RCC_AHB1ENR  = (uint32_t*)(RCC_ADDR_BASE + 0x30);
	uint32_t* GPIOD_MODER = (uint32_t*)(GPIOD_ADDR_BASE + 0x00);
//	*GPIOD_MODER |=(1 << 24) |  (1 << 26) | (01 << 28) | (1 << 30);

	MODER_t* GPIOD = (MODER_t*)0x40020c00;
	GPIOD->MODER12 = OUTPUT;
	GPIOD->MODER13 = OUTPUT;
	GPIOD->MODER14 = OUTPUT;
	GPIOD->MODER15 = OUTPUT;





}

typedef enum
{
	OFF,
	ON
}Led_state_t;

void LED_ctrl(int led_num, Led_state_t state)
{

	uint32_t* GPIOD_ODR  = (uint32_t*)(GPIOD_ADDR_BASE + 0x14);
	if(state == OFF)
		*GPIOD_ODR &= ~(1<<(13 + led_num));
	else
		*GPIOD_ODR |= (1<<(13 + led_num));
}
//#define RCC_ADDR_BASE 0x40023800
#define GPIOD_BASE_ADDRESS 0x40020C00
#define GPIOA_BASE_ADDRESS 0x40020000

void button_init()
{

/*	uint32_t* RCC_AHB1ENR = (uint32_t*)0x40023830;
	*RCC_AHB1ENR |= (1<<0);*/
	uint32_t* RCC_AHB1ENR	= (uint32_t*)(RCC_ADDR_BASE + 0x30);
	*RCC_AHB1ENR |= (1<<0);
//	__HAL_RCC_GPIOA_CLK_ENABLE();
	uint32_t* GPIOA_MODER  = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x00);
	uint32_t* GPIOA_PUPDR  = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x0C);
	*GPIOA_MODER &= ~(0b11);
	*GPIOA_PUPDR &= ~(0b11);
}
char get_button()
{

	uint32_t* GPIOA_IDR  = (uint32_t*)(GPIOA_BASE_ADDRESS + 0x10);

	return *GPIOA_IDR & 0b000001;
}

void delay(){

}

//unsigned int* MODER = 0x40020c00;


#define TIMER5_BASE 0x40000c00
uint32_t* CNT = (uint32_t*) (TIMER5_BASE + 0x24);
uint32_t* CCR1 = (uint32_t*) (TIMER5_BASE + 0x34);
uint32_t* CCR2 = (uint32_t *) (TIMER5_BASE + 0x38);
uint16_t cnt_val = 0;
uint16_t cap1_val = 0;    // capture/compare
uint16_t cap2_val = 0;



void TIM4_PWM_Init()
{
	uint32_t* RCC_AHB1ENR = (uint32_t*)0x40023830;
	*RCC_AHB1ENR |= (1<<3);
	uint32_t* GPIOD_MODER = (uint32_t*)0x40020c00;
	*GPIOD_MODER &= ~(0b11 << 24);
	*GPIOD_MODER |= (0b10 << 24);
	uint32_t* GPIOD_AFRH = (uint32_t*)0x40020c24;
	*GPIOD_AFRH |= (0b0010<<16);

	uint32_t* RCC_APB1ENR = (uint32_t*)0x40023840;
	*RCC_APB1ENR |= (1<<2);
	uint16_t* ARR = (uint16_t*)0x4000082c;
	uint16_t* PSC = (uint16_t*)0x40000828;
	uint32_t* CCR1 = (uint32_t*)0x40000834;
	*ARR = 999; //~100%
	*PSC = 16;   // từ 16MHZ có thể set lên 64 Mhz
	*CCR1 = 30;      // có thể thay đôi value 25 lên 30
	uint16_t* CCMR1 = (uint16_t*)0x40000818;
	*CCMR1 &= ~(0b11);
	//set 00: CC1 channel is configured as OUTPUT.
	*CCMR1 |= (0b110 << 4);
	//PWM mode 1

	uint16_t* CCER = (uint16_t*)0x40000820;
	*CCER |= 1;
	uint32_t* CR1 = (uint32_t*)0x40000800;
	*CR1 |= 1;
}

void Modify_PWM (uint8_t value)
{
	uint32_t* CCR1 = (uint32_t *)0x40000834;
	*CCR1 = value;

}


// doc van toc dong co
void Time5_Capture()
{
	uint32_t* RCC_AHB1ENR = (uint32_t*)0x40023830;
	*RCC_AHB1ENR |= (1<<0);
	uint32_t* MODER = (uint32_t*)0x40020000;	// GPIOA
	*MODER &= ~(0b11);
	*MODER |= (0b10);
	uint32_t* AFRL = (uint32_t*)0x40020020;		//GPIOA
	*AFRL |= 2;

	uint32_t* RCC_APB1ENR = (uint32_t*)0x40023840;
	*RCC_APB1ENR |= (1<<3);
	//timer basic
	//uint32_t* CR1 = (uint32_t*)0x40000c00;
	uint32_t* CR1 = (uint32_t*) (TIMER5_BASE);			//control register
	uint32_t* PSC = (uint32_t*) (TIMER5_BASE + 0x28);	//pre-scaler
	uint32_t* ARR = (uint32_t*) (TIMER5_BASE + 0x2C);   //auto reload register
	//ftimer (div: 1) = 16 000 000 cnt --> 1000 ms
	//(div: 16 000) = 1 cnt --> 1 ms
	*PSC = 16000 - 1;
	*ARR = 0xffff;   //65535 = ffff cua CCR1

	// timer capture Channel 1 - rising
	uint16_t* CCER = (uint16_t*) (TIMER5_BASE + 0x20);
	uint16_t* CCMR1 = (uint16_t*) (TIMER5_BASE + 0x18);
	*CCER &= ~((1<< 1) | (1<<3));					//select trigger 1 rising(len) for TI1FP1
	*CCMR1 |= 0b01;									//select TI1FP1 for IC1=InputCapture1
	*CCER |= 1;										//enable capture channel 1
	 //timer capture Channel 2 - falling
	*CCER &= ~( (1<<7));							//select trigger 1 falling(xg) for TI1FP1
	*CCER |= (1<< 5) ;
	*CCMR1 |= 0b10 << 8;							//select trigger TI1FP2 for IC2
	*CCER |= 1 << 4;								//enable capture channel 2

	// timer slave mode control reset cnt when rising
	uint16_t* SMCR = (uint16_t*) (TIMER5_BASE + 0X08);
	*SMCR |= 0b100;									//selection slave reset mode
	*SMCR |= 0b101 << 4;							//select trigger source is TI1FP1

	*CR1 |= 1;										//enable count

}

void main(void)
{



	LED_Init();
	TIM4_PWM_Init();
	Time5_Capture();
	//button_init();

	Clock_HSE_Init();
	while(1)
	{
		LED_ctrl(1, ON);
		delay(1000);

		LED_ctrl(1, OFF);
		delay(1000);






//		LED_ctrl(1, OFF);


	}
}




