#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h" // Thu vien xu ly chuoi (strcmp, memset)

// --- 1. Khai bao Handle ---
QueueHandle_t xQueue_RxData; // Queue chua tung ky tu nhan duoc tu may tinh

// --- 2. Khai bao nguyen mau ham ---
void System_Init(void);
void vTask_ProcessCmd(void *pvParameters);

// --- 3. Phan Ngat (Quan trong nhat) ---
// Ngat nay chay doc lap, ngat ngang CPU de lay du lieu ngay lap tuc
void USART1_IRQHandler(void) {
    uint8_t rx_byte;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Kiem tra co ngat nhan du lieu (RXNE flag)
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        // 1. Doc ky tu tu thanh ghi du lieu
        rx_byte = USART_ReceiveData(USART1);

        // 2. Gui vao Queue (Phien ban DUNG TRONG NGAT)
        // Luu y: KHONG duoc dung xQueueSend thuong, phai dung ...FromISR
        xQueueSendFromISR(xQueue_RxData, &rx_byte, &xHigherPriorityTaskWoken);

        // 3. Xoa co ngat (De dam bao khong bi treo ngat)
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    
    // Yeu cau chuyen ngu canh neu viec gui Queue danh thuc Task uu tien cao hon
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// --- 4. Task Xu ly Lenh (Command Processing) ---
void vTask_ProcessCmd(void *pvParameters) {
    uint8_t received_char;
    char buffer[20]; // Bo dem de ghep cac ky tu thanh chuoi
    uint8_t index = 0;

    // Xoa sach bo dem truoc khi dung
    memset(buffer, 0, 20);

    while(1) {
        // Cho nhan tung ky tu tu Queue
        // portMAX_DELAY: Task se NGU (Block) mai mai neu khong co du lieu
        if (xQueueReceive(xQueue_RxData, &received_char, portMAX_DELAY) == pdTRUE) {
            
            // Kiem tra ky tu ket thuc cau lenh (xuong dong \n hoac \r)
            if (received_char == '\n' || received_char == '\r') {
                // -> Da nhan xong 1 chuoi lenh
                buffer[index] = '\0'; // Them ky tu ket thuc chuoi
                
                // So sanh chuoi lenh voi tu khoa
                if (strcmp(buffer, "LED ON") == 0) {
                    GPIO_ResetBits(GPIOC, GPIO_Pin_13); // Bat LED (Active Low)
                } 
                else if (strcmp(buffer, "LED OFF") == 0) {
                    GPIO_SetBits(GPIOC, GPIO_Pin_13);   // Tat LED
                }
                
                // Reset bien dem va xoa bo dem de nhan lenh moi
                index = 0; 
                memset(buffer, 0, 20);
            } 
            else {
                // -> Chua xong, ghep tiep ky tu vao bo dem
                if (index < 19) { // Kiem tra tran bo dem (Buffer Overflow)
                    buffer[index++] = received_char;
                }
            }
        }
    }
}

// --- 5. Ham Main ---
int main(void) {
    // Cau hinh phan cung
    System_Init();

    // Tao Queue: Chua toi da 50 ky tu (byte)
    xQueue_RxData = xQueueCreate(50, sizeof(uint8_t));

    if (xQueue_RxData != NULL) {
        // Tao Task xu ly voi Stack 256 words
        xTaskCreate(vTask_ProcessCmd, "Process", 256, NULL, 1, NULL);

        // Bat dau bo lap lich
        vTaskStartScheduler();
    }

    while(1); // Chuong trinh khong bao gio chay den day
}

// --- 6. Cau hinh phan cung (Driver) ---
void System_Init(void) {
    // Cap Clock cho GPIO, USART
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitTypeDef gpio;

    // --- Cau hinh LED PC13 (Output) ---
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &gpio);
    
    // Tat LED ban dau (PC13 muc 1 la Tat)
    GPIO_SetBits(GPIOC, GPIO_Pin_13);

    // --- Cau hinh USART1 TX (PA9) ---
    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP; // Chan TX phai la AF Push Pull
    GPIO_Init(GPIOA, &gpio);

    // --- Cau hinh USART1 RX (PA10) ---
    // QUAN TRONG: Chan RX phai la Input Floating hoac Pull Up
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    // --- Cau hinh thong so UART ---
    USART_InitTypeDef uart;
    uart.USART_BaudRate = 9600;                 // Toc do 9600
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // Bat ca Nhan va Gui
    USART_Init(USART1, &uart);

    // --- Kich hoat Ngat Nhan (RX Interrupt) ---
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // Bat ngat khi co du lieu den
    
    // --- Cau hinh NVIC (Trinh quan ly ngat) ---
    NVIC_InitTypeDef nvic;
    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0; // Uu tien cao nhat
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // Bat UART
    USART_Cmd(USART1, ENABLE);
}