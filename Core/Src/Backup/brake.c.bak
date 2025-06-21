#include "brake.h"
#include "main.h"

extern TIM_HandleTypeDef htim4;
void brake_init(void)
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  htim4.Instance->CCR4 = 0;
}

void brake_release(void)
{
  htim4.Instance->CCR4 = 1023;
  HAL_Delay(500);
  htim4.Instance->CCR4 = 40;

}

void brake_lock(void)
{
  htim4.Instance->CCR4 = 0;
}

