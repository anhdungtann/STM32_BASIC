#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct {
    uint32_t period_ms;  
    uint32_t duty_ms;    
} LED_Config_t;

QueueHandle_t xQueue;

void GPIO_Config(void) {
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);
}

void TaskBlink(void *pvParameters) {
    LED_Config_t config;
	
    config.period_ms = 50;
    config.duty_ms = 25;

    while (1) {

        xQueueReceive(xQueue, &config, 0);

        GPIOC->ODR |= GPIO_Pin_13;
        vTaskDelay(pdMS_TO_TICKS(config.duty_ms));

        GPIOC->ODR &= ~GPIO_Pin_13;
        vTaskDelay(pdMS_TO_TICKS(config.period_ms - config.duty_ms));
    }
}

void TaskQueueSend(void *pvParameters) {
    LED_Config_t config;
    config.period_ms = 50;
    config.duty_ms = 25;

    while (1) {
			
        config.period_ms += 100;
        config.duty_ms += 60;

        if (config.period_ms > 1000) config.period_ms = 50;
        if (config.duty_ms > config.period_ms) config.duty_ms = config.period_ms / 2;

        xQueueSend(xQueue, &config, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

int main(void) {
    GPIO_Config();

    xQueue = xQueueCreate(2, sizeof(LED_Config_t));

    xTaskCreate(TaskBlink, "Blink", 128, NULL, 1, NULL);
    xTaskCreate(TaskQueueSend, "Send", 128, NULL, 2, NULL);

    vTaskStartScheduler();

    while (1);
}