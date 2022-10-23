#ifndef PWM_H
#define PWM_H
/**
 * @file pwm.h
 */

int PWM_Init(TIM_TypeDef *timer, unsigned channel, GPIO_TypeDef *gpio, unsigned pin);
int PWM_SetValue(TIM_TypeDef *timer, unsigned channel, unsigned value);
int PWM_Config(TIM_TypeDef *timer, unsigned channel, unsigned top, unsigned div, int pol);
int PWM_StopTImer(TIM_TypeDef *timer);
int PWM_StartTImer(TIM_TypeDef *timer);

#endif

