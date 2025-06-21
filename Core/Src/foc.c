#include "main.h"
#include "foc.h"
#include "IR2181.h"
#include "serial.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "header.h"


extern i32 light_tick, light_Z;

void foc_init(void)
{
  //u32 v3 = 0.95 * 8000;
  //u32 v2 = (0.95 + 0) * 8000;
  //u32 v1 = (0.95 + 0 + 0.05) * 8000;
  //IR2181_Set(v1,v2,v3);
  //HAL_Delay(200);
  i32 last_light_tick = light_tick;
  //uprintf("%lld\n", last_light_tick);
  //v2 = 0.95 * 8000;
  //v3 = (0.95 + 0) * 8000;
  //v1 = (0.95 + 0 + 0.05) * 8000;
  //IR2181_Set(v3,v1,v1);
  //HAL_Delay(200);
  //uprintf("%lld\t%lld\n", light_tick, last_light_tick - light_tick);
  i32 sub_tick = last_light_tick - light_tick;
  if(sub_tick < 0) sub_tick = -sub_tick;
  //if(sub_tick > 330 && sub_tick < 400)
  {
    light_tick = 0; 
    light_Z = 0;
  }
  HAL_Delay(200);
}

extern u16 adcDatas[2];
extern int loop_type;
float difUi1 = 0, difUi2 = 0;

float Id = 0, Iq = 0.0;
extern ADC_HandleTypeDef hadc1;

float lastV1 = 1, lastV2 = 1, lastV3 = 1;
float lastQid = 0, lastQiq = 0;
u32 printTick = 0;
i32 last_tick = 0;
float e_angle = 0, newAngle = 0;
float lastAngle = -1;
float iv = 0;
float curV = 0;
i8 dir = 1;
float lastIq = 0;

float lastDv = 0;

float setV = 0, lastSetV = 0;
float iDist = 0;
float DistLastError = 0;

float lastEAngle = 0;
float startEangle = 0;

u32 cnt = 0;
float lastui1 = -1, lastui2 = -1;
u8 last_hall_pos = 32;
float qd = 0, dd = 0;
float setP = PI2/2;
float i1=0,i2=0,i3=0;
float curAngle=0;
extern float curOnRecVP, recvP, recvV, recvI;

extern u8 controlState;
extern float hallAngleDif;

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  i32 ui1 = 0;
  i32 ui2 = 0; 
  //for(int i = 0; i < 32; ++i)
  //{
  //  ui1 += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  //  ui2 += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  //}
  //ui1 >>= 5;
  //ui2 >>= 5;

  ui1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
  ui2 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);


  float ratio = 0.5;
  if(loop_type == 0) ratio = 0.9;
  if(lastui1 < 0) lastui1 = ui1;
  else
  {
    lastui1 = lastui1 * ratio + ui1 * (1-ratio);
  }

  if(lastui2 < 0) lastui2 = ui2;
  else
  {
    lastui2 = lastui2 * ratio + ui2 * (1-ratio);
  }

  if(loop_type == 0)
  {
    difUi1 = lastui1;
    difUi2 = lastui2;
    return;
  }


  i1 =  (3.300 * (lastui1-difUi1) / 4096 / 0.066) ;
  i3 = -1 * (3.300 * (lastui2-difUi2) / 4096 / 0.066) ;
  //float i1 =  (3.3 * (lastui1-2040) / 4096 / 0.066);
  //float i3 = -1 * ((3330.0 / 4096) * (lastui2-2053) / 66);
  i2 = -(i1 + i3);
  

  //float Ialpha = i1 - 0.5*i2 - 0.5*i3;
  //float Ibeta = 0.866*i2 - 0.886*i3;

  u8 hall_pos = foc_get_hall();
  //uprintf("%d\t%d\t%d\n", hall0, curArea, curIndex);



  //e_angle = mag_getEAngle();


  if(last_hall_pos != hall_pos)
  {
    last_hall_pos = hall_pos;

    if(hall_pos == 3) e_angle = 2 * PI2 / 6 + hallAngleDif;
    else if(hall_pos == 2) e_angle = 3 * PI2 / 6 + hallAngleDif;
    else if(hall_pos == 6) e_angle = 4 * PI2 / 6 + hallAngleDif;
    else if(hall_pos == 4) e_angle = 5 * PI2 / 6 + hallAngleDif;
    else if(hall_pos == 5) e_angle = 0 * PI2 / 6 + hallAngleDif;
    else if(hall_pos == 1) e_angle = 1 * PI2 / 6 + hallAngleDif;

    lastEAngle = mag_getEAngle();
    startEangle = e_angle;
  }
  else 
  {
    //float curEanlge = mag_getEAngle();
    //float dEangle = curEanlge - lastEAngle;
    //if(dEangle < -PI) dEangle += PI2;
    //if(dEangle > PI) dEangle -= PI2;
    //e_angle = startEangle + dEangle;
  }

  float cosValue = cosf(e_angle);
  float sinValue = sinf(e_angle);
  
  float curAlpha = 0.6666667f * (i1 - 0.5 * (i2 + i3));
  float curBeta  = 0.6666667f * (i2 - i3);

  float curId = cosValue * curAlpha + sinValue * curBeta;
  float curIq = -sinValue * curAlpha + cosValue * curBeta;

  lastQiq = 0.999*lastQiq + 0.001*curIq;
  lastQid = 0.999*lastQid + 0.001*curId;

  //float curId = 0.6666667f*(cosValue*i1 + (SQRT3_2*sinValue-.5f*cosValue)*i2 + (-SQRT3_2*sinValue-.5f*cosValue)*i3);
  //float curIq = 0.6666667f*(-sinValue*i1 - (-SQRT3_2*cosValue-.5f*sinValue)*i2 - (SQRT3_2*cosValue-.5f*sinValue)*i3);

  //Iq = -1;

  float kpId = 0.0, kiId = 0.0;
  float kpIq = 30, kiIq = 0.0;

  float ed = Id - lastQid;
  float eq = Iq - lastQiq;
  dd += ed;
  qd += eq;
  if(dd > 100) dd = 100;
  if(dd < -100) dd = -100;
  if(qd > 100) qd = 100;
  if(qd < -100) qd = -100;

  float dId = kpId * ed + kiId * dd;
  float dIq = kpIq * eq + kiIq * qd;

  dId = 0;
  dIq = Iq;
  float norm = sqrtf(dId * dId + dIq * dIq);
  if(norm > 36)
  {
      dId = dId * 36/norm;
      dIq = dIq * 36/norm;
  }

  #define targetAngle1 0
  #define targetAngle2 0
  float target_e_angle = e_angle + targetAngle1;  
  if(dIq > 0) target_e_angle = e_angle + targetAngle2;  
  
  //target_e_angle = PI2 * (printTick % 1000) / 1000.0;
  //if(Iq < 0) target_e_angle =  e_angle - PI/2;

  cosValue = cosf(target_e_angle);
  sinValue = sinf(target_e_angle);

  float u = cosValue*dId - sinValue*dIq;
  float v = (SQRT3_2*sinValue-.5f*cosValue)*dId - (-SQRT3_2*cosValue-.5f*sinValue)*dIq;
  float w = (-SQRT3_2*sinValue-.5f*cosValue)*dId - (SQRT3_2*cosValue-.5f*sinValue)*dIq;

  float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))*0.5f;
  float v_midpoint = .5f*(0.94+0);
  //v_offset = 0;
  float dtc_u = fast_fminf(fast_fmaxf((1.0*(u - v_offset)*1/36 + v_midpoint ), 0), 0.94);
  float dtc_v = fast_fminf(fast_fmaxf((1.0*(v - v_offset)*1/36 + v_midpoint ), 0), 0.94);
  float dtc_w = fast_fminf(fast_fmaxf((1.0*(w - v_offset)*1/36 + v_midpoint ), 0), 0.94);

  u32 v1 = dtc_u * 8000;
  u32 v2 = dtc_v * 8000;
  u32 v3 = dtc_w * 8000;

  //v1 = 100;v2 = 700, v3 = 800;
  IR2181_Set(v1,v2,v3);

  u32 minV = (v1 < v2) ? (v1 < v3 ? v1 : v3) : (v2 < v3 ? v2 : v3);
  u32 maxV = (v1 > v2) ? (v1 > v3 ? v1 : v3) : (v2 > v3 ? v2 : v3);
  lastV1 = (maxV - minV) / 8000.0;


  curAngle = mag_getAngle();
  if(printTick % 100 == 0)
  {
    //float curAngle = 360.0 * mag_getRawAngle() / PI2;
    //float cureAngle = 360.0 * mag_getEAngle() / PI2 ;
    //float curLAngle = 360.0 * angle_getAngle() / PI2 ;
    //float curSAngle = 360.0 * mag_getSingleAngle() / PI2 ;
    //i32 curtick = angle_tick();

    //uprintf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", e_angle/10,curAlpha,curBeta, lastQiq, lastQid);
    //uprintf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", e_angle, lastQiq, lastQid, dIq, qd);

    //uprintf("%.3f\n",sqrtf(i1*i1+i2*i2+i3*i3));
    //uprintf("%.3f\t%.3f\t%.3f\t%.3f\n",e_angle, i1, i2, i3);

    //uprintf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t\n", i1, i2, i3, curIq, curId);
    //uprintf("%.3f\t%.3f\t%.3f\t%.3f\t\n", curAngle, setV, curV, Iq );


        //uprintf("%.2f\t%.2f\t%.2f\n", 360*e_angle/PI2, cureAngle,  curAngle);
        //uprintf("%d\t%d\t%.2f\n", curtick, hall_pos, curSAngle);

    //uprintf("%.1f\t%.1f\t%.1f\t%.1f\t%.1f\n",curLAngle, curSAngle, curAngle,  360*e_angle/PI2, cureAngle);

  }
  //if(printTick % 1000 == 0)
  //{
  //  e_angle += PI/3;
  //  if(e_angle >= PI2) 
  //  {
  //    e_angle -= PI2;
  //    cnt++;
  //  }
  //}
  printTick++;

  if(lastAngle < 0)
  {
    lastAngle = curAngle;
  }

  
  if(printTick % 5 == 0)   //  2ms 通过V计算q
  {
    
    float distance =  (curAngle - lastAngle);
    if(distance > PI)  distance = PI2 - distance;
    if(distance < -PI)  distance = -PI2 - distance;

    curV = distance / 0.002;
    lastAngle = curAngle;

    

    float absSetV = setV;
    float dv = absSetV - curV;

    //printf("%f\n", dv);
    if(fabsf(dv) > 2)
    {
      dv /= 5;
      //if(dv > 0) dv = 0.1;
      //else dv = -0.1;
    }
  
    iv += dv;


    if(iv > PI/2) iv = PI/2;
    else if(iv < -PI/2) iv = -PI/2;
    float kpv = 6, kiv = 0, kdv = 200;
    float dq = kpv * (dv) + kiv * iv + kdv * (dv - lastDv);
    Iq += dq;
 
    lastIq = 0.99 * lastIq + (1-0.99) * Iq;
    Iq = lastIq;
    lastDv = dv;

    if(Iq > recvI) Iq = recvI;
    else if(Iq < -recvI) Iq = -recvI;
  }

  if(printTick % 10 == 0)   //  20ms   通过d计算v
  {
    float n = floorf(recvP / PI2);
    recvP = recvP - n * PI2;
    float distance1 = (recvP - curAngle);

    //float startP = curOnRecVP;
    //float distance2 = (curAngle - startP);
    //if(distance2 > 0) distance2 += PI / 6;
    //else distance2 -= PI/6;

    float distance = distance1;
    //if(fabsf(distance1) > fabsf((recvP - startP) / 2))
    //{
    //  distance = distance2;
    //  if(distance1 > 0 && distance2 < 0) distance = -distance2;
    //  else if(distance1 < 0 && distance2 > 0) distance = -distance2;
    //}


    
    if(fabsf(distance) < PI_2)  controlState = 0;

    if(controlState == 0)
    {
      if(distance > PI_2*3)  distance = distance - PI2;
      if(distance < -PI_2*3)  distance = PI2 + distance;
    }

    iDist += distance;
    if(iDist > PI/3) iDist = PI/3;
    else if(iDist < -PI/3) iDist = -PI/3;
    float dDistenceError = distance - DistLastError;
    float kpd = 5, kid = .005, kdd = 1100;
    float d_setV = kpd * distance + kid * iDist + kdd * dDistenceError;


    
    lastSetV = 0.9*lastSetV + (1-0.9)*d_setV;
    setV = lastSetV;
    if(setV < 0.01 && setV > -0.01) setV = 0;
    else 
    {
      if(setV > recvV) setV = recvV;
      else if(setV < -recvV) setV = -recvV;
    }
    DistLastError = distance;
    //if(printTick % 500 == 0) 
    //  printf("%f\t%f\t%f\n", distance, d_setV, setV);
    //e_angle+=PI/3;
  } 
  //setP = PI/1;
  //setV = 2;
    //Iq = 5;

   //if(printTick % 1000 == 0)
   //{
   // uprintf2("hello\n");
   //}
}


u8 foc_get_hall(void)
{
  u8 hall0 = HAL_GPIO_ReadPin(hall0_GPIO_Port, hall0_Pin);
  hall0 |= (HAL_GPIO_ReadPin(hall1_GPIO_Port, hall1_Pin)) << 1;
  hall0 |= (HAL_GPIO_ReadPin(hall2_GPIO_Port, hall2_Pin)) << 2;
  return hall0;
}