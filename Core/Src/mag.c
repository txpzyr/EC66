
#include "header.h"
#include <math.h>

//i32 avg_angle = -1;
i64 turnCount = 0;
i8 magDir = 0;
float magLastAngle = 9999;
float multiTurn = 0;

float mag_getRawAngle(void)
{
    u8 values[4];
    mu_sdad_transmission(values, 4);
    //u32 angle = ((values[0]&0x07) << 24) |  (values[1] << 16) | (values[2] << 8) | values[3];
    u32 angle = (values[1] << 16) | (values[2] << 8) | values[3];
    //angle >>= 3;
    //if(avg_angle < 0)  
    //avg_angle = angle;
    //else avg_angle = 0.1 * angle + 0.9 * avg_angle;
    return PI2 * (angle / 16777215.0);
}

float mag_getAngle(void)
{
  float rawAngle = mag_getRawAngle();
  if(magLastAngle > PI2)  //初始化
  {
    multiTurn = magLastAngle = rawAngle;
  }
  else
  {
    if(fabsf(magLastAngle - rawAngle) > PI)
    {
      if(magLastAngle > rawAngle) 
      {
        magDir = 1;
        turnCount++;
      }
      else if(magLastAngle < rawAngle) 
      {
        magDir = -1;
        turnCount--;
      }
      else 
      {
      }
    }
  }
  multiTurn = turnCount * PI2 + rawAngle;
  magLastAngle = rawAngle;
  return multiTurn;
}

float mag_getSingleAngle(void)
{
    float fAngle = mag_getRawAngle();
    fAngle *= 80;
    int n = floorf(fAngle / PI2);
    fAngle -= PI2*n;
    return fAngle;
}

float mag_getEAngle(void)
{
    float fAngle = mag_getRawAngle();
    fAngle *= 80;
    fAngle = fAngle * 5 * 1;
    int n = floorf(fAngle / PI2);
    fAngle -= PI2*n;
    return fAngle;
}