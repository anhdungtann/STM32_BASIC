#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO

GPIO_InitTypeDef gpio;
GPIO_InitTypeDef led;
// ham delay 
void delay(int time){
    int i,j;
    for(i = 0; i < time; i++){
        for(j = 0; j < 0x2aff; j++);
    }
}

void config(){
    // cap xung clock cho ngoai vi
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // cau hinh nut nhan PA6 IPU 
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &gpio);
    
    // cau hinh led PA1 output push pull
    led.GPIO_Mode = GPIO_Mode_Out_PP;
    led.GPIO_Pin = GPIO_Pin_1;
    led.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &led);
    
    // mac dinh led tat (PA1 = 1)
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
}

int main(){
    config(); 
    uint8_t sttOld = 1;
		uint8_t sttNew = 1;
    uint16_t counter = 1; 
    while(1){
			sttOld = sttNew;
			sttNew = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6); // doc nut PA6
        // phat hien canh xuong (tu 1 -> 0 nghia la nhan nut)
        if(sttOld == 1 && sttNew == 0){ 
            counter++; 
            
            if(counter % 2 == 0){ 
                GPIO_ResetBits(GPIOA, GPIO_Pin_1); // LED sang
            } else { 
                GPIO_SetBits(GPIOA, GPIO_Pin_1);   // LED tat
            }
            
            delay(2); // debounce
        } 
    }
}
