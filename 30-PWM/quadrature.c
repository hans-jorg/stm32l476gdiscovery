/**
 * @file    quadrature.c
 * @brief   HAL for a position encoder with quadrature outputs
 *
 * @version 1.0
 *
 * @note   There are two implementation. One, more precise, reading the
 *         A and B inputs and decoding it with a state machine. The other
 *         uses one input as a clock and the other as a direction signal.
 *
 * @note   Both inputs must be on the same GPIO port
 *
 * @note   It uses a debouncing algorithm. At start, it needs N cycles to stabilize.
 *
 */


#include  <stdint.h>

#include "gpio2.h"
#include "quadrature.h"

/*
 * The following timers have support for quadrature decoding
 *    * Advanced control timers (TIM1/TIM8)
 *    * General purpose timers (TIM2/TIM3/TIM4/TIM5)
 *    * Low power timer (LPTIM)

 *   A and B signal must be connected to CH1 and CH2 inputs of a timer.
 *
 * Timer  | CH1   | CH2   |  AF    |     Obs
 * -------|-------|-------|--------|-------------------
 *  TIM1  | PA8   | PA9   |  AF1   |
 *  TIM1  | PE9   | PE11  |  AF1   |
 *  TIM8  | PC6   | PC7   |  AF3   |
 *  TIM2  | PA0   | PA1   |  AF1   |
 *  TIM2  | PA15  | PB3   |  AF1   | There is a CH1 on PA5
 *  TIM3  | PA6   | PA7   |  AF2   |
 *  TIM3  | PB4   | PB5   |  AF2   |
 *  TIM3  | PC6   | PC7   |  AF2   |
 *  TIM3  | PE3   | PE4   |  AF2   |
 *  TIM4  | PB6   | PB7   |  AF2   |   <------
 *  TIM4  | PD12  | PD13  |  AF2   |
 *  TIM5  | PF6   | PF7   |  AF2   |
 *  TIM5  | PA0   | PA1   |  AF2   |
 *
 *  On the STM32L476 Discovery Board most of these pins are already used.
 *  But, PB6 and PB7 are on pins 15 and 16 of header P1.
 *  They are labeled I2C1_SCL and I2C1_SDA.
 *  Others will conflick with LED or Joystick.

 */

#define USING_PULSE_DIR_SIGNALS
//#define USING_A_B_SIGNALS


typedef struct {
    TIM_TypeDef     *timer;
    GPIO_TypeDef    *gpio1;
    uint16_t         pin1;
    uint16_t         af1;
    GPIO_TypeDef    *gpio2;
    uint16_t         pin2;
    uint16_t         af2;
} TIM_PINS;

static const TIM_PINS pintable[] = {
/*              timer  gpio1  pin1  af1  gpio2  pin2   af2 */
/*  0  */    {   TIM1, GPIOA,   8,   1,  GPIOA,   9 ,   1   },
/*  1  */    {   TIM1, GPIOE,   9,   1,  GPIOE,  11,    1   },
/*  2  */    {   TIM2, GPIOA,   0,   1,  GPIOA,   1 ,   1   },
/*  3  */    {   TIM2, GPIOA,  15,   1,  GPIOB,   3 ,   1   }, //There is a CH1 on PA5
/*  4  */    {   TIM3, GPIOA,   6,   2,  GPIOA,   7,    2   },
/*  5  */    {   TIM3, GPIOB,   4,   2,  GPIOB,   5,    2   },
/*  6  */    {   TIM3, GPIOC,   6,   2,  GPIOC,   7,    2   },
/*  7  */    {   TIM3, GPIOE,   3,   2,  GPIOE,   4,    2   },
/*  8  */    {   TIM4, GPIOB,   6,   2,  GPIOB,   7,    2   }, // Use this
/*  9  */    {   TIM4, GPIOD,  12,   2,  GPIOD,  13,    2   },
/* 10  */    {   TIM5, GPIOF,   6,   2,  GPIOF,   7,    2   },
/* 11  */    {   TIM5, GPIOA,   0,   2,  GPIOA,   1,    2   },
/* 12  */    {   TIM8, GPIOC,   6,   3,  GPIOC,   7,    3   },
/* END */    {      0,     0,   0,   0,      0,   0,    9   }
};

// Default
#if !defined(USING_A_B_SIGNALS) && !defined(USING_PULSE_DIR_SIGNALS)
#define USING_A_B_SIGNALS
#endif

/*****************************************************************************
 * @brief  Quadrature decoder
 *****************************************************************************/

///@{
#define QUADRATURE_TIMER            TIM4
#define QUADRATURE_GPIO1            GPIOB
#define QUADRATURE_PIN1             (6)
#define QUADRATURE_AF1              (2)
#define QUADRATURE_GPIO2            GPIOB
#define QUADRATURE_PIN2             (7)
#define QUADRATURE_AF2              (2)

#define QUADRATURE_GPIOBUTTON       GPIOH
#define QUADRATURE_PINBUTTON        (0)
///@}


#define QUADRATURE_MASK1            (1U<<QUADRATURE_PIN1)
#define QUADRATURE_MASK2            (1U<<QUADRATURE_PIN2)
#define QUADRATURE_BUTTON       (1U<<QUADRATURE_PINBUTTON)

static void Timer_EnableClock(TIM_TypeDef *timer) {
uint32_t m = 0;

    if     ( timer == TIM1 ) m = 0;
    else if( timer == TIM2 ) m = RCC_APB1ENR1_TIM2EN;
    else if( timer == TIM3 ) m = RCC_APB1ENR1_TIM3EN;
    else if( timer == TIM4 ) m = RCC_APB1ENR1_TIM4EN;
    else if( timer == TIM5 ) m = RCC_APB1ENR1_TIM5EN;
    else if( timer == TIM6 ) m = RCC_APB1ENR1_TIM6EN;
    else if( timer == TIM7 ) m = RCC_APB1ENR1_TIM7EN;
    else if( timer == TIM8 ) m = 0;
    else m = 0;

    RCC->APB1ENR1 |= m;
}




// These routines are identical for each deboding method

void Quadrature_Init(void) {

    /* Configure pins as inputs */
    GPIO_Init(QUADRATURE_GPIO1,QUADRATURE_MASK1,0);
    GPIO_Init(QUADRATURE_GPIO2,QUADRATURE_MASK2,0);
    GPIO_Init(QUADRATURE_GPIOBUTTON,QUADRATURE_BUTTON,0);

    // Pullup for button
    GPIO_ConfigurePins(QUADRATURE_GPIOBUTTON,QUADRATURE_BUTTON,GPIO_CONF_IN|GPIO_CONF_UP);

    // Configuration for quadrature inputs
    GPIO_ConfigurePins(QUADRATURE_GPIO1,QUADRATURE_MASK1,GPIO_CONF_IN);
    GPIO_ConfigurePins(QUADRATURE_GPIO1,QUADRATURE_MASK2,GPIO_CONF_IN);

    // Enable clock for timer
    Timer_EnableClock(QUADRATURE_TIMER);

    // Disable it
    QUADRATURE_TIMER->CR1 &= ~TIM_CR1_CEN;

        // Set polarity
    QUADRATURE_TIMER->CCER &= ~(TIM_CCER_CC1P|TIM_CCER_CC2P);

    // Set filter
    QUADRATURE_TIMER->CCER &= ~(TIM_CCER_CC1NP|TIM_CCER_CC2NP);

    // Set upper counter
    QUADRATURE_TIMER->ARR = -1;

    //CC1S=01,CC2S=01

    // Set Encoder Interface by making SMS = 0b011
    // Alternatives are:
    //              0b001 count on T2 edges
    //              0b010 count on T1 edges
    //              0b011 count on both T1 and T2 edges

    QUADRATURE_TIMER->SMCR = (QUADRATURE_TIMER->SMCR&~(TIM_SMCR_SMS))|(3<<TIM_SMCR_SMS_Pos);

    // Enable it
    QUADRATURE_TIMER->CR1 |= TIM_CR1_CEN;

}

int Quadrature_GetPosition(void) {

    return (int16_t) QUADRATURE_TIMER->CNT&0xFFFF;
}

void Quadrature_Load(int v) {

    QUADRATURE_TIMER->CNT = v;

}

void Quadrature_Reset(void) {

    QUADRATURE_TIMER->CNT = 0;
}


int Quadrature_GetButtonStatus(void) {

    return (~GPIO_Read(QUADRATURE_GPIOBUTTON))&QUADRATURE_BUTTON;
}

