#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO

GPIO_InitTypeDef led;
GPIO_InitTypeDef button;

void config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	//cau hinh led gpio 1A
	led.GPIO_Mode = GPIO_Mode_Out_PP;
	led.GPIO_Pin = GPIO_Pin_1;
	led.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&led);
	//cau hinh nut nhan gpio 6a
	button.GPIO_Mode = GPIO_Mode_IPU;
	button.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA,&button);
}
void delay(int time){
	while(time--);
}

int main(){
	config();
	while(1){
		uint8_t status = GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 0){
			GPIO_WriteBit(GPIOA,GPIO_Pin_1,!status);
			delay(50000);
		}
	}
}


