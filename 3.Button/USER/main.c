#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO

GPIO_InitTypeDef gpio;

void config(){
	/*cap xung clock cho ngoai vi*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB, ENABLE);
	
	/*cau hinh nut nhan PB9*/
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);
	
	/*cau hinh cho led PC13*/
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_13;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpio);
}

int main(){
	config();
	while(1){//nhan nut
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0){
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);//den sang
		}
		else{//nha nut
			GPIO_SetBits(GPIOC, GPIO_Pin_13);//den tat		
		}
	}
}
