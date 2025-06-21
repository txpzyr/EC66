
#include "main.h"
#include "angle.h"


#define LIGHT_ROUND 4250
#define ANGLE_PER_TICK 0.00469599135716446  // PI2 / (LIGHT_ROUND / 5)
//#define ANGLE_PER_TICK 0.0  // PI2 / (LIGHT_ROUND / 5)

i32 light_tick = 0,  light_Z = 0;
i64 last_light_tick = 0, sum_light_tick = 0;

extern float e_angle;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
    case light_A_Pin:
      if(HAL_GPIO_ReadPin(light_B_GPIO_Port, light_B_Pin) == GPIO_PIN_SET)
      {
        last_light_tick=sum_light_tick;
        sum_light_tick--;
        light_tick--;
        e_angle -= ANGLE_PER_TICK;
      }
      else
      {
        last_light_tick=sum_light_tick;
        sum_light_tick++;
        light_tick++;
        e_angle += ANGLE_PER_TICK;
      }
      break;
    case light_B_Pin:
      if(HAL_GPIO_ReadPin(light_A_GPIO_Port, light_A_Pin) == GPIO_PIN_RESET) 
      {
        last_light_tick=sum_light_tick;
        sum_light_tick--;
        light_tick--;
        e_angle -= ANGLE_PER_TICK;
      }
      else
      {
        last_light_tick=sum_light_tick;
        sum_light_tick++;
        light_tick++;
        e_angle += ANGLE_PER_TICK;
      }
      break;
    //case light_Z_Pin:
    //  if(last_light_tick > light_tick) 
    //  {
    //    light_Z++;
    //    light_tick = 0;
    //  }
    //  else 
    //  {
    //    light_Z--;
    //    light_tick = LIGHT_ROUND;
    //  }
    //  break;
    default:
      break;
  }

  //if(e_angle < 0) e_angle = PI2 + e_angle;
  //else if(e_angle > PI2) e_angle = e_angle - PI2;

  //if(light_tick >= LIGHT_ROUND) 
  //{
  //  light_tick = 0;
  //  //light_Z++;
  //}
  //if(light_tick < 0)
  //{
  //  light_tick = LIGHT_ROUND - 1;
  //  //light_Z--;
  //}
  
}

i32 angle_tick(void)
{
  //u32 tick_per_pair = light_tick % CODER_ROUND_PER_PAIR;
  return light_tick;
}

float angle_getAngle(void)
{
  //u32 tick_per_pair = light_tick % CODER_ROUND_PER_PAIR;
  return PI2 * (light_tick % LIGHT_ROUND) / LIGHT_ROUND;
}

float angle_get_e_angle(void)
{
  u32 tick_per_pair = light_tick % CODER_ROUND_PER_PAIR;
  return PI2 * tick_per_pair / CODER_ROUND_PER_PAIR;
}