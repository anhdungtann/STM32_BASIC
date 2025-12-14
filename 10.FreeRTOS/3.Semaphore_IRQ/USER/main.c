#include "stm32f10x.h"                  // Device header

#include "FreeRTOS.h"                   // RTOS:Core&&Cortex-M

#include "semphr.h"                     // RTOS:Core&&Cortex-M

#include "task.h"                       // RTOS:Core&&Cortex-M

#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC

#include "stm32f10x_usart.h"            // Device:StdPeriph Drivers:USART

#include "stm32f10x_exti.h"             // Device:StdPeriph Drivers:EXTI

#include "queue.h"                      // RTOS:Core&&Cortex-M



SemaphoreHandle_t mySema;





void gpio_config(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);

	GPIO_InitTypeDef gpio;

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;

	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;

	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA,&gpio);

	

	gpio.GPIO_Mode = GPIO_Mode_IPU;

	gpio.GPIO_Pin = GPIO_Pin_2;

	GPIO_Init(GPIOA,&gpio);

}



void config_interrupt(){

	EXTI_InitTypeDef exti;

	NVIC_InitTypeDef nvic;

	

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

	exti.EXTI_Line = EXTI_Line2;

	exti.EXTI_LineCmd = ENABLE;

	exti.EXTI_Trigger = EXTI_Trigger_Falling;

	EXTI_Init(&exti);

	

	nvic.NVIC_IRQChannel = EXTI2_IRQn;

	nvic.NVIC_IRQChannelCmd = ENABLE;

	nvic.NVIC_IRQChannelPreemptionPriority = 6;

	nvic.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&nvic);

	

}



void EXTI2_IRQHandler(void){

	BaseType_t flag_check = pdFALSE;

	if(EXTI_GetITStatus(EXTI_Line2) != RESET){

		xSemaphoreGiveFromISR(mySema, &flag_check);

		EXTI_ClearITPendingBit(EXTI_Line2);

		portYIELD_FROM_ISR(flag_check);

	}

}



void task_led_control(void *pvParameters){

	while(1){

		GPIOA -> ODR ^= GPIO_Pin_0;

		vTaskDelay(pdMS_TO_TICKS(500));

	}

}



void task_button(void *pvParameters){

	while(1){

		if(xSemaphoreTake(mySema, portMAX_DELAY) == pdTRUE){

			GPIO_ResetBits(GPIOA, GPIO_Pin_1);

			vTaskDelay(pdMS_TO_TICKS(1000));

			GPIO_SetBits(GPIOA, GPIO_Pin_1);

		}

	}

}



int main (){

		gpio_config();

		config_interrupt();

		

		mySema = xSemaphoreCreateBinary();

		

		xTaskCreate(task_led_control,"task1",128,NULL,1,NULL);

		xTaskCreate(task_button,"task2",128,NULL,2,NULL);

		

		vTaskStartScheduler();

	while(1){



	}

}