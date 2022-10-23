#ifndef QUADRATURE_H
/** ***************************************************************************
 * @file    quadrature.h
 * @brief   HAL for a quadrature encoder
 * @version 1.0
******************************************************************************/
#define QUADRATURE_H
#include <stdint.h>

#define QUADRATURE_LPTIMER

void    Quadrature_Init(TIM_TypeDef *timer,int cha, int chb, int mode);
int32_t Quadrature_GetPosition(TIM_TypeDef *timer);
void    Quadrature_Reset(TIM_TypeDef *timer);
void    Quadrature_Load(TIM_TypeDef *timer, int32_t v);
void    Quadrature_SetMaximum(TIM_TypeDef *timer, int32_t v);

#define QUADRATURE_MODE_X4          (0)
#define QUADRATURE_MODE_X2_A        (1)
#define QUADRATURE_MODE_X2_B        (2)

#ifdef QUADRATURE_LPTIMER
void    Quadrature_LPInit(LPTIM_TypeDef *timer,int cha, int chb, int mode);
int32_t Quadrature_LPGetPosition(LPTIM_TypeDef *timer);
void    Quadrature_LPReset(LPTIM_TypeDef *timer);
void    Quadrature_LPLoad(LPTIM_TypeDef *timer, int32_t v);
void    Quadrature_LPSetMaximum(LPTIM_TypeDef *timer, int32_t v);
#endif

#endif // QUADRATURE_H
