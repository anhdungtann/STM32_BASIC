#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_exti.h"             // Device:StdPeriph Drivers:EXTI
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM
#include "misc.h"                       // Device:StdPeriph Drivers:Framework

void config_led(){
	//cau hinh gpio
	GPIO_InitTypeDef led;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	led.GPIO_Mode = GPIO_Mode_Out_PP;
	led.GPIO_Speed = GPIO_Speed_50MHz;
	led.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC,&led);
}

void config_timer_interrupt(){
	//cau hinh timer
	TIM_TimeBaseInitTypeDef timer;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	timer.TIM_CounterMode = TIM_CounterMode_Up;
	timer.TIM_ClockDivision = 0;
	timer.TIM_Period = 49999;
	timer.TIM_Prescaler = 719;
	timer.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&timer);
	//ngat update timer
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	//cau hinh nvic
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);
	//bat timer
	TIM_Cmd(TIM2,ENABLE);
}

void TIM2_IRQHandler(){
	if(TIM_GetITStatus(TIM2,TIM_IT_Update != RESET)){
		static uint8_t state = 0;
		state = !state;
		GPIO_WriteBit(GPIOC,GPIO_Pin_13, state);
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}

int main(){
	config_led();
	config_timer_interrupt();
	while(1){
	}
}
