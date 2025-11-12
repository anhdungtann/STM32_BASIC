#include "stm32f10x.h"                  // Device header
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:R 

GPIO_InitTypeDef GPIO;
//cau hinh gpio c13
void config(){
	GPIO.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO.GPIO_Pin = GPIO_Pin_13;
	GPIO.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOC, &GPIO);
}
void delay(int time){
	int i,j;
		for(i=0;i<=time;i++){
			for(j=0;j<=0x2aff;j++){}
	}
}
int main(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); // CAP CLOCK CHO GPIOC
	config();
	while(1){
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		delay(1000);
		GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		delay(1000);
	}
}