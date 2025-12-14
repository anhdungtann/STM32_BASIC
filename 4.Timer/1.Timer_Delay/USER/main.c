#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_exti.h"             // Device:StdPeriph Drivers:EXTI
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM
#include "misc.h"                       // Device:StdPeriph Drivers:Framework

void config_led(){
	GPIO_InitTypeDef led;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	led.GPIO_Mode = GPIO_Mode_Out_PP;
	led.GPIO_Speed = GPIO_Speed_50MHz;
	led.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC,&led);
}

void config_timer(){
	TIM_TimeBaseInitTypeDef timer;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	timer.TIM_ClockDivision = 0;
	timer.TIM_Period = 49999;
	timer.TIM_Prescaler = 719;
	timer.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&timer);
	TIM_Cmd(TIM2,ENABLE);
}

void delay(int time){
	int i;
	for(i=0;i<time;i++){
		TIM_SetCounter(TIM2,0);
		while(TIM_GetCounter(TIM2)<100);
	}	
}

int main(){
	config_led();
	config_timer();
	while(1){
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		delay(500);
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		delay(500);
	}
}