#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stdio.h" // Thu vien dung cho ham sprintf

// --- 1. Khai bao Handle cho Queue ---
QueueHandle_t xQueue_ADC_Data;

// --- 2. Khai bao nguyen mau ham ---
void System_Init(void);
void UART_SendString(char* str);

// --- 3. Cac Tac vu (Tasks) ---

// Tac vu 1: Doc cam bien (Producer)
// Chu ky: 100ms
void vTask_Sensor(void *pvParameters) {
    uint16_t adc_val;

    while(1) {
        // Bat dau chuyen doi ADC (Software Trigger)
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
        
        // Cho chuyen doi xong (EOC = End Of Conversion)
        // Luu y: ADC STM32F1 rat nhanh nen dung while o day chap nhan duoc
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
        
        // Doc gia tri ADC tu thanh ghi
        adc_val = ADC_GetConversionValue(ADC1);

        // Gui vao Queue
        // Tham so 3 = 0: Neu Queue day thi bo qua luon, khong cho
        xQueueSend(xQueue_ADC_Data, &adc_val, 0);

        // Delay 100ms (Nhuong CPU cho Task khac)
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Tac vu 2: Hien thi/Gui UART (Consumer)
// Hoat dong: Chi chay khi co du lieu trong Queue
void vTask_Display(void *pvParameters) {
    uint16_t received_val;
    char buffer[50];

    while(1) {
        // Nhan du lieu tu Queue
        // portMAX_DELAY: Task se NGU (Block) mai mai den khi co du lieu trong Queue
        if (xQueueReceive(xQueue_ADC_Data, &received_val, portMAX_DELAY) == pdTRUE) {
            
            // Xu ly du lieu: Doi so nguyen sang chuoi ky tu
            sprintf(buffer, "Gia tri ADC: %d\r\n", received_val);
            
            // Gui chuoi qua UART len may tinh
            UART_SendString(buffer);
        }
    }
}

// --- 4. Ham Main ---
int main(void) {
    // Cau hinh phan cung (Clock, GPIO, ADC, UART)
    System_Init();

    // Tao Queue: Chua toi da 5 phan tu, kich thuoc moi phan tu la uint16_t
    xQueue_ADC_Data = xQueueCreate(5, sizeof(uint16_t));

    // Kiem tra neu tao Queue thanh cong
    if (xQueue_ADC_Data != NULL) {
        // Tao Task 1: Stack 128 words, Uu tien 2 (Cao hon de uu tien doc du lieu)
        xTaskCreate(vTask_Sensor, "Sensor", 128, NULL, 2, NULL);

        // Tao Task 2: Stack 256 words (lon hon do dung sprintf), Uu tien 1
        xTaskCreate(vTask_Display, "Display", 256, NULL, 1, NULL);

        // Bat dau bo lap lich FreeRTOS
        vTaskStartScheduler();
    }

    while(1); // Chuong trinh khong bao gio chay den day
}

// --- 5. Cac ham cau hinh phan cung (Driver SPL) ---
void System_Init(void) {
    // --- Cap Clock cho cac ngoai vi ---
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1 | RCC_APB2Periph_USART1, ENABLE);
    
    // Cap Clock cho ADC (Max 14MHz -> 72MHz / 6 = 12MHz)
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); 

    GPIO_InitTypeDef gpio;
    
    // --- Cau hinh PA0 (ADC Input - Chan bien tro) ---
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_AIN; // Che do Analog Input
    GPIO_Init(GPIOA, &gpio);

    // --- Cau hinh PA9 (UART TX - Chan truyen) ---
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push Pull
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    // --- Cau hinh ADC1 ---
    ADC_InitTypeDef adc;
    adc.ADC_Mode = ADC_Mode_Independent;        // Che do doc lap
    adc.ADC_ScanConvMode = DISABLE;             // Khong dung che do quet (chi 1 kenh)
    adc.ADC_ContinuousConvMode = DISABLE;       // Khong chuyen doi lien tuc (Trigger bang tay)
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // Trigger bang phan mem
    adc.ADC_DataAlign = ADC_DataAlign_Right;    // Can le phai (12 bit)
    adc.ADC_NbrOfChannel = 1;                   // So luong kenh: 1
    ADC_Init(ADC1, &adc);
    
    // Cau hinh chi tiet: Kenh 0, Rank 1, Thoi gian lay mau 55.5 chu ky
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    
    ADC_Cmd(ADC1, ENABLE); // Bat ADC

    // --- Hieu chuan ADC (Bat buoc voi STM32F1) ---
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1)); // Cho reset hieu chuan xong
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));      // Cho hieu chuan xong

    // --- Cau hinh UART1 ---
    USART_InitTypeDef uart;
    uart.USART_BaudRate = 9600;                 // Toc do baud 9600
    uart.USART_WordLength = USART_WordLength_8b;// 8 bit du lieu
    uart.USART_StopBits = USART_StopBits_1;     // 1 bit stop
    uart.USART_Parity = USART_Parity_No;        // Khong kiem tra chan le
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx;            // Chi cau hinh chuc nang truyen (TX)
    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);                  // Bat UART
}

// Ham gui chuoi ky tu qua UART
void UART_SendString(char* str) {
    while(*str) {
        USART_SendData(USART1, *str);
        // Cho den khi qua trinh truyen hoan tat (Co TXE duoc set)
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
        str++;
    }
}