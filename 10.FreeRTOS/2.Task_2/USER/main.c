#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC


#include "FreeRTOS.h"                   // RTOS:Core&&Cortex-M
#include "task.h"                       // RTOS:Core&&Cortex-M

typedef struct{
	uint8_t pin;
	unsigned int frequency;
} ledconfig_t;

void GPIO_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);
}


void vTaskblink(void *pvParameters){
	ledconfig_t *led = (ledconfig_t*)pvParameters;
	TickType_t Tickdelay = pdMS_TO_TICKS(1000/led->frequency/2);
	while(1){
		GPIO_ResetBits(GPIOA,led->pin);
		vTaskDelay(Tickdelay);
		GPIO_SetBits(GPIOA,led->pin);
		vTaskDelay(Tickdelay);
	}
}

int main(){
	
	GPIO_Config();
	
	static ledconfig_t led1 = {GPIO_Pin_1,3};
	static ledconfig_t led2 = {GPIO_Pin_2,10};
	static ledconfig_t led3 = {GPIO_Pin_3,25};
	
	xTaskCreate(vTaskblink,"LED1",128,&led1,1,NULL);
	xTaskCreate(vTaskblink,"LED2",128,&led2,1,NULL);
	xTaskCreate(vTaskblink,"LED3",128,&led3,1,NULL);
	vTaskStartScheduler();
	while(1){
	}
}
