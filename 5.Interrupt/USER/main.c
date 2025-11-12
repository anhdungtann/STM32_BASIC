#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x.h"                  // Device header
#include "misc.h"                       // Device:StdPeriph Drivers:Framework
#include "stm32f10x_exti.h"             // Device:StdPeriph Drivers:EXTI

GPIO_InitTypeDef led;
GPIO_InitTypeDef button;
//ham delay
void delay(int time){
	int i,j;
	for(i=0;i<time;i++){
		for(j=0;j<0x2aff;j++);
	}
}
//cau hinh gpio
void config_GPIO(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	//cau hinh led Pin A1, A2;
	led.GPIO_Mode = GPIO_Mode_Out_PP;
	led.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	led.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&led);
	//cau hinh nut nhan Pin A6;
	button.GPIO_Mode = GPIO_Mode_IPU;
	button.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA,&button);
}
//cau hinh exti
void config_EXTI(){
	EXTI_InitTypeDef exti;
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource6);
	exti.EXTI_Line = EXTI_Line6;
	exti.EXTI_LineCmd = ENABLE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&exti);
}
//cau hinh nvic
void config_NVIC(){
	NVIC_InitTypeDef nvic;
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);
}

void EXTI9_5_IRQHandler(){    
	static uint16_t counter = 0;
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == 0){
		counter++;      
		if(counter % 2 == 0){ 
			GPIO_ResetBits(GPIOA, GPIO_Pin_2); 
		} 
		else{ 
			GPIO_SetBits(GPIOA, GPIO_Pin_2); 
		}
	}         
	EXTI_ClearITPendingBit(EXTI_Line6);
}

int main(){
	config_GPIO();
	config_NVIC();
	config_EXTI();
	while(1){
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
		delay(500);
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
		delay(500);
	}
}
