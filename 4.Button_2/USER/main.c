#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Keil::Device:StdPeriph Drivers:GPIO

GPIO_InitTypeDef gpio;

// ham delay 
void delay(int time){
    int i,j;
    for(i = 0; i < time; i++){
        for(j = 0; j < 0x2aff; j++);
    }
}

void config(){
    // cap xung clock cho ngoai vi
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB, ENABLE);
    
    // cau hinh nut nhan PB9 IPU 
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    
    // cau hinh led PB11 output push pull
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    
    // mac dinh led tat (PB11 = 1)
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
}

int main(){
    config();
    
    uint8_t sttOld = 1, sttNew = 1; 
    uint16_t counter = 1; 
    
    while(1){
        sttOld = sttNew; 
        sttNew = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9); // doc nut PB9
        
        // phat hien canh xuong (tu 1 -> 0 nghia la nhan nut)
        if(sttOld == 1 && sttNew == 0){ 
            counter++; 
            
            if(counter % 2 == 0){ 
                GPIO_ResetBits(GPIOB, GPIO_Pin_11); // LED sang
            } else { 
                GPIO_SetBits(GPIOB, GPIO_Pin_11);   // LED tat
            }
            
            delay(2); // debounce
        } 
    }
}
