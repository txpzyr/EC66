#include "header.h"
#include <math.h>
#include <stdio.h>
#include "control.h"


extern i8 ID;
extern struct RecvData recvData;
//extern float setP, setV, Iq;
extern float curAngle, curV, Iq, setV, setP;
extern float i1,i2,i3;
u8 controlState = 0;

float curOnRecVP = 0, recvP = 0, recvV = 0, recvI = 0;

u8 answeredMotor = 0;

extern u16 recLen;


// 预计算CRC表
uint16_t crc_table[256];

void init_crc_table(void) {
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
        crc_table[i] = crc;
    }
}

void InitControl(void)
{
  init_crc_table();
}

uint16_t calculate_crc_fast(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < len; i++) {
        uint8_t index = (crc >> 8) ^ data[i];
        crc = (crc << 8) ^ crc_table[index];
    }
    
    return crc;
}

void ProcessCommand(void)
{
  //printf("%d\n", recvData.id);
  if(recvData.prefix[0] != 0xEA || recvData.prefix[1] != 0x9E) return;
  u16 recvCRC = recvData.crc;
  recvData.crc = 0;
  u16 curCRC = calculate_crc_fast((u8*)&recvData, recLen);
  if(curCRC != recvCRC) return;

  if(recvData.type == 2)
  {
    if(recvData.id == 0xEF || recvData.id == ID-1)
    {
        curOnRecVP = mag_getAngle();
        recvP = recvData.P[ID-1];
        controlState = 1;
        recvV = recvData.V[ID-1];
        recvI = recvData.I[ID-1];

        answeredMotor = 0;
    }
    else
    {
      answeredMotor |= (1<<recvData.id);
      //printf("2\n");
      HAL_Delay(30);
    }
    if(recvData.id == ID-1 || answeredMotor == ((1<<(ID-1)) - 1))
    {
        struct SendData sd;
        sd.header[0] = 0xEA;
        sd.header[1] = 0x9E;
        sd.id = ID-1;
        sd.type = 2;
        sd.I = sqrtf(i1*i1+i2*i2+i3*i3)/2;
        sd.V = curV;
        sd.P = curAngle;
        sd.crc = 0;
        sd.crc = calculate_crc_fast((u8*)&sd, sizeof(sd));
        usend((uint8_t *)&sd, sizeof(sd));
        
        HAL_Delay(50);
    }
  }
  else if(recvData.type == 1)
  {
    
    if(recvData.id == ID-1)
    {
      //printf("3\n");

      struct SendData sd;
      sd.header[0] = 0xEA;
      sd.header[1] = 0x9E;
      sd.id = ID-1;
      sd.type = 1;
      sd.I = sqrtf(i1*i1+i2*i2+i3*i3)/2;
      sd.V = curV;
      sd.P = curAngle;
      sd.crc = 0;
      sd.crc = calculate_crc_fast((u8*)&sd, sizeof(sd));
      usend((uint8_t *)&sd, sizeof(sd));
      HAL_Delay(50);
      //uprintf2("setV:%.3f, startP:%.3f\n", setV, curOnRecVP*180/PI);
    }
  }
  else if(recvData.type == 3)
  {
      if(recvData.id == 0xEF) 
      {
        answeredMotor = 0;
        //printf("1\n");
      }
      else 
      {
        answeredMotor |= (1<<recvData.id);
        //printf("2\n");
        HAL_Delay(30);
      }
      if(answeredMotor == ((1<<(ID-1)) - 1))
      {
        //printf("3\n");

        struct SendData sd;
        sd.header[0] = 0xEA;
        sd.header[1] = 0x9E;
        sd.id = ID-1;
        sd.type = 3;
        sd.I = sqrtf(i1*i1+i2*i2+i3*i3)/2;
        sd.V = curV;
        sd.P = curAngle;
        sd.crc = 0;
        sd.crc = calculate_crc_fast((u8*)&sd, sizeof(sd));
        usend((uint8_t *)&sd, sizeof(sd));
        answeredMotor = 0;
        HAL_Delay(50);
        //uprintf2("setV:%.3f, startP:%.3f\n", setV, curOnRecVP*180/PI);
      }
  }
}