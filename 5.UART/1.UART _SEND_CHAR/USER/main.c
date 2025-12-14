#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_usart.h"            // Device:StdPeriph Drivers:USART
#include "string.h"
#include "stdio.h"

void config_gpio(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA,&gpio);
}

void config_uart(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_InitTypeDef usart;
	usart.USART_BaudRate = 9600;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Tx;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2,&usart);
	USART_Cmd(USART2,ENABLE);	
}
void uart_SendChar(char c){
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET); // neu co du lieu truyen trong thi co =1; khong trong (co du lieu) = 0;
	USART_SendData(USART2, c);
}

void uart_SendStr(char *str){
	while(*str != '\0'){
		uart_SendChar(*str++);
	}
}

void Delay_ms(unsigned int t){
	unsigned int i, j;
	for(i = 0; i < t; i++){
		for(j = 0; j < 0x2aff; j++);
	}
}

int main(){
	config_gpio();
	config_uart();
	while(1){
		uart_SendStr("Hello from STM32!\n");
		Delay_ms(500);
	}
}