
#ifndef __SERIAL_H
#define __SERIAL_H
#include "main.h"

#pragma pack(1)
struct RecvData
{
  u8 prefix[2];
  u8 id;
  u8 type;
  u16 crc;
  float I[6];
  float V[6];
  float P[6];
  
};

struct SendData
{
  u8 header[2];
  u8 id;
  u8 type;
  u16 crc;

  float I;
  float V;
  float P;
};

struct SendString
{
  u8 header[2];
  u8 id;
  u8 type;
  u16 crc;
  char msg[64];
};
#pragma pack()

void SerialInit(void);
void uprintf(char *fmt,...);
int Serial_Available(void);
char* Serial_ReadString(void);
int Serial_Peek(void) ;
int Serial_PeekNextDigit(uint8_t detectDecimal);
float Serial_ParseFloat(void);
void Serial_Receive(void);
void usend(u8* datas, u16 len);
void uprintf2(char *fmt,...) ;

#endif