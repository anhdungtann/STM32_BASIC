#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h" 						// Keil::Device:StdPeriph Drivers:GPIO

GPIO_InitTypeDef led;
GPIO_InitTypeDef button;
void Config_led(){
	led.GPIO_Mode = GPIO_Mode_Out_PP;
	led.GPIO_Pin = GPIO_Pin_13;
	led.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &led);
}

void Config_button(){
	button.GPIO_Mode = GPIO_Mode_IPU;
	button.GPIO_Pin	= GPIO_Pin_6;
	GPIO_Init(GPIOA, &button);
}


int main(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	Config_led();
	Config_button();
	while(1){
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 0){
			GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		}
		else{
			GPIO_SetBits(GPIOC,GPIO_Pin_13);
		}
	}
}
