#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h" // Keil::Device:StdPeriph Drivers:GPIO

GPIO_InitTypeDef GPIO;
void Config(){
	GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO.GPIO_Pin = GPIO_Pin_13;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO);
}
	
void delay(int time){
	int i,j;
	for(i = 0; i<time;i++){
		for(j = 0; j<0x2aff;j++);
	}
}

int main(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	
	Config();
	while(1){
		GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay(500);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay(500);
	}
}
