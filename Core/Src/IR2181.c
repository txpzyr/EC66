#include "main.h"
#include "IR2181.h"

extern TIM_HandleTypeDef htim1;

void IR2181_init(void)
{
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  htim1.Instance->CCR1 = 0;
  htim1.Instance->CCR2 = 0;
  htim1.Instance->CCR3 = 0;
}

void IR2181_Set(u32 u1, u32 u2, u32 u3)
{
  htim1.Instance->CCR1 = u1;
  htim1.Instance->CCR2 = u2;
  htim1.Instance->CCR3 = u3;
}