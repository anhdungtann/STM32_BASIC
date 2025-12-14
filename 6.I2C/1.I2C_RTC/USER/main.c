#include "stm32f10x.h"                  // Device header
#include "stm32f10x_i2c.h"              // Device:StdPeriph Drivers:I2C
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO
#include "stm32f10x_usart.h"            // Device:StdPeriph Drivers:USART
#include "stm32f10x_tim.h"              // Device:StdPeriph Drivers:TIM
#include "string.h"
#include "stdio.h"

/* Dinh nghia dia chi I2C cua DS1307
   Dia chi goc la 0x68. Trong SPL, can dich trai 1 bit: 0x68 << 1 = 0xD0
*/
#define DS1307_ADDR 0xD0

void uart_config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Pin = GPIO_Pin_9;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpio.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA,&gpio);
	
	USART_InitTypeDef uart;
	uart.USART_BaudRate = 9600;
	uart.USART_WordLength = USART_WordLength_8b;
	uart.USART_StopBits = USART_StopBits_1;
	uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart.USART_Parity = USART_Parity_No;
	uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1,&uart);
	
	USART_Cmd(USART1,ENABLE);
}

void USARTx_SendChar(char c){
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET); // thanh ghi du lieu truyen khong trong thi truyen.
	USART_SendData(USART1, c);
}


struct __FILE { 
    int dummy; 
}; 
 
FILE __stdout; 
  
int fputc(int ch, FILE *f) { 
    USARTx_SendChar(ch); 
    return ch; 
}

void I2C_Config(void) {
    I2C_InitTypeDef  I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Cap clock cho I2C1 va GPIOB */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* Cau hinh chan PB6 (SCL) va PB7 (SDA) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    /* QUAN TRONG: Che do AF_OD (Open Drain) */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Cau hinh I2C */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000; // 100kHz
    
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}




void timer_config(){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_TimeBaseInitTypeDef time;
	time.TIM_ClockDivision = 0;
	time.TIM_CounterMode = TIM_CounterMode_Up;
	time.TIM_Period = 65535;
	time.TIM_Prescaler = 72 - 1;
	time.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&time);
	TIM_Cmd(TIM2,ENABLE);
}

void delay_ms(int time){
	while(time--){
		TIM_SetCounter(TIM2,0);
		while(TIM_GetCounter(TIM2)<1000);
		}
}

uint8_t gio, phut, giay;

/* Ham chuyen doi BCD sang Decimal (Thap phan) */
uint8_t BCD2DEC(uint8_t data) {
    return (data >> 4) * 10 + (data & 0x0F);
}

/* Ham doc thoi gian tu DS1307 */
void DS1307_ReadTime(uint8_t *h, uint8_t *m, uint8_t *s) {
    
    /* --- PHAN 1: GHI DIA CHI THANH GHI MUON DOC (0x00) --- */

    /* Cho I2C ranh (Busy flag = 0) */
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

    /* Tao xung START */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Cho su kien Master Mode duoc chon (EV5) */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    /* Gui dia chi Slave (0xD0) de GHI (Transmitter) */
    I2C_Send7bitAddress(I2C1, DS1307_ADDR, I2C_Direction_Transmitter);
    /* Cho su kien Master gui xong va san sang (EV6) */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Gui dia chi thanh ghi bat dau cua DS1307 (0x00 - Giay) */
    I2C_SendData(I2C1, 0x00);
    /* Cho byte duoc gui di thanh cong (EV8) */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* --- PHAN 2: DOC DU LIEU VE --- */

    /* Tao xung START lan 2 (Repeated Start) */
    I2C_GenerateSTART(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    /* Gui dia chi Slave (0xD0) de NHAN (Receiver) */
    I2C_Send7bitAddress(I2C1, DS1307_ADDR, I2C_Direction_Receiver);
    /* Cho su kien Master san sang nhan (EV6) */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* --- PHAN 3: LAY DU LIEU TU BUFFER --- */

    /* Doc GIAY: Can doc tiep nen gui ACK */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t raw_s = I2C_ReceiveData(I2C1);

    /* Doc PHUT: Can doc tiep nen gui ACK */
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t raw_m = I2C_ReceiveData(I2C1);

    /* Doc GIO: Byte cuoi cung -> KHONG gui ACK (NACK) va gui STOP */
    I2C_AcknowledgeConfig(I2C1, DISABLE); // Tat ACK
    I2C_GenerateSTOP(I2C1, ENABLE);       // Chuan bi STOP
    
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
    uint8_t raw_h = I2C_ReceiveData(I2C1);

    /* Bat lai ACK cho cac lan sau */
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    /* --- PHAN 4: XU LY BIT VÀ CHUYEN DOI --- */
    
    /* Loai bo bit CH (bit 7) o thanh ghi Giay */
    *s = BCD2DEC(raw_s & 0x7F);
    
    /* Phut khong co bit dieu khien */
    *m = BCD2DEC(raw_m);
    
    /* Loai bo cac bit mode 12h/24h (bit 6) o thanh ghi Gio */
    *h = BCD2DEC(raw_h & 0x3F);
}

int main(void){
    /* Khoi tao cac ngoai vi */
    uart_config();
    timer_config();
    I2C_Config(); // Bat buoc phai co
    
    printf("DS1307 Test Started...\n");

    while (1)
    {
        /* Doc thoi gian */
        DS1307_ReadTime(&gio, &phut, &giay);
        
        /* In ra Serial Monitor */
        printf("Time: %02d:%02d:%02d\n", gio, phut, giay);
        
        /* Doi 1 giay */
        delay_ms(1000);
    }
}