#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO
#include "stm32f10x_exti.h"             // Keil::Device:StdPeriph Drivers:EXTI

void config_gpio(){
	GPIO_InitTypeDef led;
	GPIO_InitTypeDef button;
	 //cap clock gpioc va gpiob
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB , ENABLE);
	// cau hinh led
	led.GPIO_Mode = GPIO_Mode_Out_PP;
	led.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_13;
	led.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &led);
	//cau hinh button
	button.GPIO_Mode = GPIO_Mode_IPU;
	button.GPIO_Pin = GPIO_Pin_9;
	button.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &button);	
}

void config_nvic(){
	NVIC_InitTypeDef nvic;
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic);
}

void config_exti(){
	EXTI_InitTypeDef exti;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);
	exti.EXTI_Line = EXTI_Line9;
	exti.EXTI_LineCmd = ENABLE;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	
	EXTI_Init(&exti);
}

void delay(int time){
	int i,j;
	for(i=0; i<=time; i++){
		for(j=0; j<=0x2aff; j++){
		}
	}
}

void EXTI9_5_IRQHandler(void){
    if(EXTI_GetITStatus(EXTI_Line9) != RESET){
        delay(50);
        uint8_t status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15);
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 0){
            GPIO_WriteBit(GPIOC, GPIO_Pin_15, !status);
        }
        
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

int main(){
	config_gpio();
	config_nvic();
	config_exti();
	GPIO_WriteBit(GPIOC, GPIO_Pin_15, 1);
	while(1){
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);
		delay(500);
		GPIO_SetBits(GPIOC, GPIO_Pin_13);
		delay(500);
	}
}
