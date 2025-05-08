/*
 * FS_IA10B.c
 *
 *  Created on: Apr 6, 2025
 *      Author: PhamThanhBien
 */

#include "FS_IA10B.h"


 FS_IA10B_ibus ibus;
 
 unsigned char ibus_check_CS(unsigned char* data, unsigned char len)
 
 {
     unsigned short CS = 0xffff;
 
     for (int i = 0; i < len-2; i++)
     {
         CS = CS - data[i];
     }
 
     return ((CS&0x00ff) == data[30]) && ((CS>>8) == data[31]);
 }
 
 void ibus_message(unsigned char* data, FS_IA10B_ibus* ibus)
 {
     ibus -> right_horizontal = (data[2] | data[3] << 8) & 0x0fff;
     ibus -> right_vertical = (data[4] | data[5] << 8) & 0x0fff;
     ibus -> left_horizontal = (data[6] | data[7] << 8) & 0x0fff;
     ibus -> left_vertical = (data[8] | data[9] << 8) & 0x0fff;
 
     ibus -> swa = (data[10] | data[11] << 8) & 0x0fff;
     ibus -> swc = (data[12] | data[13] << 8) & 0x0fff;
     ibus -> swd = (data[14] | data[15] << 8) & 0x0fff;
 
     ibus -> failsafe = ibus -> swd == 1500;							//so sánh giá trị tại switch D có bằng 1500 hay không
}
 
unsigned char ibus_active_failsafe(FS_IA10B_ibus* ibus)
{
     return ibus -> failsafe != 0;
}
 
 float map_ibus_to_altitude(unsigned short ibus_value) {

    float min_in = 1000.0f;
    float max_in = 2000.0f;
    float min_out = 0.0f;
    float max_out = 100.0f; //cm

    if (ibus_value < min_in) ibus_value = min_in;
    if (ibus_value > max_in) ibus_value = max_in;

    return ((float)(ibus_value - min_in)) * (max_out - min_out) / (max_in - min_in) + min_out; //cm
 }


 void FS_IA10B_USART1_initialization(void)
 {
     /* USER CODE BEGIN USART1_Init 0 */
 
       /* USER CODE END USART1_Init 0 */
 
       LL_USART_InitTypeDef USART_InitStruct = {0};
 
       LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
 
       /* Peripheral clock enable */
       LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
 
       LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
       /**USART1 GPIO Configuration
       PA9   ------> USART1_TX
       PA10   ------> USART1_RX
       */
       GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
       GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
       GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
       GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
       GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
       GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
       LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
       /* USART1 interrupt Init */
       NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
       NVIC_EnableIRQ(USART1_IRQn);
 
       /* USER CODE BEGIN USART1_Init 1 */
 
       /* USER CODE END USART1_Init 1 */
       USART_InitStruct.BaudRate = 115200;
       USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
       USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
       USART_InitStruct.Parity = LL_USART_PARITY_NONE;
       USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
       USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
       USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
       LL_USART_Init(USART1, &USART_InitStruct);
       LL_USART_ConfigAsyncMode(USART1);
       LL_USART_Enable(USART1);
       /* USER CODE BEGIN USART1_Init 2 */
 
       /* USER CODE END USART1_Init 2 */
 }
 
