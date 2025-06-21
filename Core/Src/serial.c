
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "header.h"

#define RECV_BUFFER_LEN 255
//uint8_t recvStr[RECV_BUFFER_LEN] = {0};
char receive_buffer[RECV_BUFFER_LEN] = {0};
#define RX_BUFFER_LEN 255
char rxBuffer[RX_BUFFER_LEN] = {0};
uint16_t rxEnd = 0;
uint16_t rxStart = 0;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;

struct RecvData recvData;
u8 hasSerialData = 0;
u16 recLen = 0;

extern u8 ID;

void SerialInit(void)
{
  //__HAL_UART_CLEAR_IDLEFLAG(&huart1);
  //__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
  //HAL_UART_Receive_DMA(&huart1, (u8*)receive_buffer, RECV_BUFFER_LEN);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL; // 循环模式
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart1_rx);

    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);
    HAL_UART_Receive_DMA(&huart1, (u8*)receive_buffer, RECV_BUFFER_LEN);
}

void uprintf(char *fmt,...) 
{
  va_list ap;
  char string[512];
  
  HAL_GPIO_WritePin(rs485_dir_GPIO_Port, rs485_dir_Pin, GPIO_PIN_SET);
  va_start(ap,fmt);
  vsprintf(string,fmt,ap);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)string, strlen(string));
  va_end(ap);
  while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC));
  HAL_GPIO_WritePin(rs485_dir_GPIO_Port, rs485_dir_Pin, GPIO_PIN_RESET);
}

void usend(u8* datas, u16 len) 
{
  HAL_GPIO_WritePin(rs485_dir_GPIO_Port, rs485_dir_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit_DMA(&huart1, datas, len);
  while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC));
  HAL_GPIO_WritePin(rs485_dir_GPIO_Port, rs485_dir_Pin, GPIO_PIN_RESET);
}

void uprintf2(char *fmt,...) 
{
  va_list ap;
  struct SendString ss;

  ss.id = ID;
  ss.type = 4;
  
  va_start(ap,fmt);
  vsprintf(ss.msg,fmt,ap);
  va_end(ap);

  u16 sendLen = strlen(ss.msg) + 4;
  ss.header[0] = 0xEA;
  ss.header[1] = 0x9E;
  HAL_GPIO_WritePin(rs485_dir_GPIO_Port, rs485_dir_Pin, GPIO_PIN_SET);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&ss, sendLen);
  while(!__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC));
  HAL_GPIO_WritePin(rs485_dir_GPIO_Port, rs485_dir_Pin, GPIO_PIN_RESET);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
     // 清除空闲中断标志
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    
    // 停止当前DMA传输
    HAL_UART_DMAStop(&huart1);

    uint32_t cndtr = hdma_usart1_rx.Instance->NDTR;
    rxEnd = (RECV_BUFFER_LEN - cndtr) % RECV_BUFFER_LEN;

    //// 设置超时时间（例如2个字节传输时间）
    //uint32_t timeout = (160000000 / huart1.Init.BaudRate)*10 + 1; // 2字节时间（微秒）
    //uint32_t start_time = HAL_GetTick();
    
    //// 等待可能后续数据
    //while((HAL_GetTick() - start_time) < timeout)
    //{
    //    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE))
    //    {
    //        // 重置超时计时器
    //        start_time = HAL_GetTick();
            
    //        // 读取新数据
    //        uint8_t temp = huart1.Instance->DR;
    //        receive_buffer[rxEnd++] = temp;
            
    //        // 检查缓冲区溢出
    //        if(rxEnd >= RECV_BUFFER_LEN) break;
    //    }
    //}

    HAL_UART_Receive_DMA(&huart1, (u8*)receive_buffer, RECV_BUFFER_LEN);
    printf("%d\n", rxEnd);

    if(rxEnd > rxStart && (rxEnd - rxStart == sizeof(struct RecvData) || rxEnd - rxStart == sizeof(struct SendData))) 
    {
      hasSerialData = 1;
      recLen = rxEnd - rxStart;
    }
    else if(rxEnd < rxStart && ( rxEnd + (RECV_BUFFER_LEN - rxStart) == sizeof(struct RecvData) || rxEnd + (RECV_BUFFER_LEN - rxStart) == sizeof(struct SendData))) 
    {
       hasSerialData = 1;
      recLen = rxEnd + (RECV_BUFFER_LEN - rxStart);
    }
}

int Serial_Available(void) {
  //return ((unsigned int) (RECV_BUFFER_LEN + rxEnd - rxStart)) % RECV_BUFFER_LEN;
  if(hasSerialData == 1)
  {
    hasSerialData = 0;
    return 1;
  }
  else
  {
    return 0;
  }
}

void Serial_Receive(void)
{
  if(rxEnd > rxStart) memcpy((u8*)&recvData, receive_buffer+rxStart, rxEnd-rxStart);
  else 
  {
    memcpy((u8*)&recvData, receive_buffer+rxStart, RECV_BUFFER_LEN-rxStart);
    memcpy((u8*)&recvData+RECV_BUFFER_LEN-rxStart, receive_buffer, rxEnd);
  }
  rxStart =  0;//(rxEnd) % RECV_BUFFER_LEN;
}

int Serial_Read(void) {
  if (Serial_Available()) {
    uint8_t uc = receive_buffer[rxStart];
	rxStart = (rxStart + 1) % RECV_BUFFER_LEN;
    return uc;
  } 
  else 
  {
    return -1;
  }
}


char* Serial_ReadString(void)
{
  int c = Serial_Read();
  int i = 0;
  while (c >= 0)
  {
    rxBuffer[i++] = c;
    c = Serial_Read();
  }
  rxBuffer[i] = 0;
  return rxBuffer;
}

int Serial_Peek(void) {
  if (Serial_Available()) {
    return receive_buffer[rxStart];
  } else {
    return -1;
  }
}


int Serial_PeekNextDigit(uint8_t detectDecimal)
{
  int c;
  while (1) {
    c = Serial_Peek();

    if( c < 0 ||
        c == '-' ||
        (c >= '0' && c <= '9') ||
        (detectDecimal && c == '.')) return c;

    //switch( lookahead ){
    //    case SKIP_NONE: return -1; // Fail code.
    //    case SKIP_WHITESPACE:
    //        switch( c ){
    //            case ' ':
    //            case '\t':
    //            case '\r':
    //            case '\n': break;
    //            default: return -1; // Fail code.
    //        }
    //    case SKIP_ALL:
    //        break;
    //}
    Serial_Read();  // discard non-numeric
  }
}


// as parseInt but returns a floating point value
float Serial_ParseFloat(void)
{
  uint8_t isNegative = 0;
  uint8_t isFraction = 0;
  long value = 0;
  int c;
  float fraction = 1.0;

  c = Serial_PeekNextDigit(1);
    // ignore non numeric leading characters
  if(c < 0)
    return 0; // zero returned if timeout

  do{
    //if(c == ignore)
    //  ; // ignore
    if(c == '-')
      isNegative = 1;
    else if (c == '.')
      isFraction = 1;
    else if(c >= '0' && c <= '9')  {      // is c a digit?
      value = value * 10 + c - '0';
      if(isFraction)
         fraction *= 0.1;
    }
    Serial_Read();  // consume the character we got with peek
    c = Serial_Peek();
  }
  while( (c >= '0' && c <= '9')  || (c == '.' && !isFraction) );

  if(isNegative)
    value = -value;
  if(isFraction)
    return value * fraction;
  else
    return value;
}