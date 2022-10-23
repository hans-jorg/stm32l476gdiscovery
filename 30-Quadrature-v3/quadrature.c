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
| Signal     |  Type    | Size  | Port/Pin  | Port/Pin  | Port/Pin  | Port/Pin  |
|------------|----------|-------|-----------|-----------|-----------|-----------|
| TIM1_CH1   | Advanced |   16  | PA8(AF1)  | PE9(AF1)  |           |           |
| TIM1_CH1N  | Advanced |   16  | PA7(AF1)  | PB13(AF1) | PE8(AF1)  |           |
| TIM1_CH2   | Advanced |   16  | PA9(AF1)  | PE11(AF1) |           |           |
| TIM1_CH2N  | Advanced |   16  | PB0(AF1)  | PB14(AF1) | PE10(AF1) |           |
| TIM8_CH1   | Advanced |   16  | PC6(AF3)  |           |           |           |
| TIM2_CH1   | Full     |   32  | PA0(AF1)  | PA5(AF1)  | PA15(AF1) |           |
| TIM2_CH2   | Full     |   32  | PA2(AF1)  | PB3(AF1)  |           |           |
| TIM3_CH1   | Full     |   16  | PA6(AF2)  | PB4(AF2)  | PC6(AF2)  | PE3(AF2)  |
| TIM3_CH2   | Full     |   16  | PA7(AF2)  | PB5(AF2)  | PC7(AF2)  | PE4(AF2)  |
| TIM4_CH1   | Full     |   16  | PB6(AF2)  | PD12(AF2) |           |           |
| TIM4_CH2   | Full     |   16  | PB7(AF2)  | PD13(AF2) |           |           |
| TIM5_CH1   | Full     |   32  | PA0(AF2)  | PF6(AF2)  |           |           |
| TIM5_CH2   | Full     |   32  | PA1(AF2)  | PF7(AF2)  |           |           |
| TIM8_CH1N  | Advanced |   16  | PA5(AF3)  | PA7(AF3)  |           |           |
| TIM8_CH2   | Advanced |   16  | PC7(AF3)  |           |           |           |
| TIM8_CH2N  | Advanced |   16  | PB0(AF3)  | PB1(AF3)  |           |           |
| LPTIM1_IN1 | Low Power|   16  | PB5(AF1)  | PC0(AF1)  | PG10(AF1) |           |
| LPTIM1_IN2 | Low Power|   16  | PB7(AF1)  | PC2(AF1)  | PG11(AF1) |           |
 *
 *  On the STM32L476 Discovery Board most of these pins are already used.
 *  But, PB6 and PB7 are on pins 15 and 16 of header P1.
 *  They are labeled I2C1_SCL and I2C1_SDA.
 *  Others will conflick with LED or Joystick.
 */

/*
 *typedef struct
 *{
 *  __IO uint32_t ISR;         // LPTIM Interrupt and Status register,      Address offset: 0x00
 *  __IO uint32_t ICR;         // LPTIM Interrupt Clear register,           Address offset: 0x04
 *  __IO uint32_t IER;         // LPTIM Interrupt Enable register,          Address offset: 0x08
 *  __IO uint32_t CFGR;        // LPTIM Configuration register,             Address offset: 0x0C
 *  __IO uint32_t CR;          // LPTIM Control register,                   Address offset: 0x10
 *  __IO uint32_t CMP;         // LPTIM Compare register,                   Address offset: 0x14
 *  __IO uint32_t ARR;         // LPTIM Autoreload register,                Address offset: 0x18
 *  __IO uint32_t CNT;         // LPTIM Counter register,                   Address offset: 0x1C
 *  __IO uint32_t OR;          // LPTIM Option register,                    Address offset: 0x20
 *} LPTIM_TypeDefXXXXX;
 *
 *typedef struct
 *{
 *  __IO uint32_t CR1;         // TIM control register 1,                   Address offset: 0x00
 *  __IO uint32_t CR2;         // TIM control register 2,                   Address offset: 0x04
 *  __IO uint32_t SMCR;        // TIM slave mode control register,          Address offset: 0x08
 *  __IO uint32_t DIER;        // TIM DMA/interrupt enable register,        Address offset: 0x0C
 *  __IO uint32_t SR;          // TIM status register,                      Address offset: 0x10
 *  __IO uint32_t EGR;         // TIM event generation register,            Address offset: 0x14
 *  __IO uint32_t CCMR1;       // TIM capture/compare mode register 1,      Address offset: 0x18
 *  __IO uint32_t CCMR2;       // TIM capture/compare mode register 2,      Address offset: 0x1C
 *  __IO uint32_t CCER;        // TIM capture/compare enable register,      Address offset: 0x20
 *  __IO uint32_t CNT;         // TIM counter register,                     Address offset: 0x24
 *  __IO uint32_t PSC;         // TIM prescaler,                            Address offset: 0x28
 *  __IO uint32_t ARR;         // TIM auto-reload register,                 Address offset: 0x2C
 *  __IO uint32_t RCR;         // TIM repetition counter register,          Address offset: 0x30
 *  __IO uint32_t CCR1;        // TIM capture/compare register 1,           Address offset: 0x34
 *  __IO uint32_t CCR2;        // TIM capture/compare register 2,           Address offset: 0x38
 *  __IO uint32_t CCR3;        // TIM capture/compare register 3,           Address offset: 0x3C
 *  __IO uint32_t CCR4;        // TIM capture/compare register 4,           Address offset: 0x40
 *  __IO uint32_t BDTR;        // TIM break and dead-time register,         Address offset: 0x44
 *  __IO uint32_t DCR;         // TIM DMA control register,                 Address offset: 0x48
 *  __IO uint32_t DMAR;        // TIM DMA address for full transfer,        Address offset: 0x4C
 *  __IO uint32_t OR1;         // TIM option register 1,                    Address offset: 0x50
 *  __IO uint32_t CCMR3;       // TIM capture/compare mode register 3,      Address offset: 0x54
 *  __IO uint32_t CCR5;        // TIM capture/compare register5,            Address offset: 0x58
 *  __IO uint32_t CCR6;        // TIM capture/compare register6,            Address offset: 0x5C
 *  __IO uint32_t OR2;         // TIM option register 2,                    Address offset: 0x60
 *  __IO uint32_t OR3;         // TIM option register 3,                    Address offset: 0x64
 *} TIM_TypeDef;
 */

/** Quadrature encoders can generate two types of signals:
 * * A and B raw signals
 * * Pluse and Dir signals obtained internally from AB signals
 */
#define USING_PULSE_DIR_SIGNALS
//#define USING_A_B_SIGNALS


/**
 * @brief   This data structure contains information about the pins used by a specific timer
 *
 * @note    The data must be ordered by timer, channel, port and pin!!!
 *
 */
typedef struct {
    void            *timer;     // Either TIM_TypeDef * or LPTIM_TypeDef *
    int              chn;       // 1 or 2
    GPIO_TypeDef    *gpio;      // GPIO
    uint16_t         pin;       // Pin
    uint16_t         af;        // Alternate function
} TIM_PININFO;

#ifdef STM32L476xx
static const TIM_PININFO pintable[] = {
/* timer  ch   gpio   pin    af   */

// Configuration for TIM1 channel 1
{   TIM1, 1, GPIOA,   8,    1   },
{   TIM1, 1, GPIOE,   9,    1   },
// Configuration for TIM1 channel 2
{   TIM1, 2, GPIOA,   9 ,   1   },
{   TIM1, 2, GPIOE,  11,    1   },

// Configuration for TIM2 channel 1
{   TIM2, 1, GPIOA,   0,    1   },
{   TIM2, 1, GPIOA,   5,    1   },
{   TIM2, 1, GPIOA,  15,    1   },
// Configuration for TIM2 channel 2
{   TIM2, 2, GPIOA,   2,    1   },
{   TIM2, 2, GPIOB,   3,    1   },

// Configuration for TIM3 channel 1
{   TIM3, 1, GPIOA,   6,    2   },
{   TIM3, 1, GPIOB,   4,    2   },
{   TIM3, 1, GPIOC,   6,    2   },
{   TIM3, 1, GPIOE,   3,    2   },
// Configuration for TIM3 channel 2
{   TIM3, 2, GPIOA,   7,    2   },
{   TIM3, 2, GPIOB,   5,    2   },
{   TIM3, 2, GPIOC,   7,    2   },
{   TIM3, 2, GPIOE,   4,    2   },

// Configuration for TIM4 channel 1
{   TIM4, 1, GPIOB,   6,    2   }, // Free on STGM32L476 Discovery
{   TIM4, 1, GPIOD,  12,    2   },
// Configuration for TIM4 channel 2
{   TIM1, 2, GPIOB,   7,    2   }, // Free on STGM32L476 Discovery
{   TIM1, 2, GPIOD,  13,    2   },

// Configuration for TIM5 channel 1
{   TIM5, 1, GPIOA,   0,    2   },
{   TIM5, 1, GPIOF,   6,    2   },
// Configuration for TIM5 channel 2
{   TIM5, 2, GPIOA,   1 ,   2   },
{   TIM5, 2, GPIOF,   6 ,   2   },

// Configuration for TIM8 channel 1
{   TIM8, 1, GPIOC,   6,    3   },
// Configuration for TIM4 channel 2
{   TIM8, 2, GPIOC,   7,    3   },

// End of table
{      0, 0,     0,   0,    0   }
};


static const TIM_PININFO lppintable[] = {
/* timer  ch   gpio   pin    af   */
// Configuration for LPTIM1 channel 1
{ LPTIM1, 1, GPIOB,   5,    1   },
{ LPTIM1, 1, GPIOC,   0,    1   },
{ LPTIM1, 1, GPIOG,  10,    1   },
// Configuration for LPTIM1 channel 2
{ LPTIM1, 2, GPIOB,   7,    1   },
{ LPTIM1, 2, GPIOC,   2,    1   },
{ LPTIM1, 2, GPIOG,  11,    1   },
{      0, 0,     0,   0,    0   }
};

#endif


/*****************************************************************************
 * @brief  Quadrature decoder
 *****************************************************************************/

/**
 * @brief   Enable clock for timers
 *
 * For all timers, including timers that do not support quadrature encoders
 */
static void Timer_EnableClock(TIM_TypeDef *timer) {

// Advanced timers
    if     ( timer == TIM1 ) RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;
    else if( timer == TIM8 ) RCC->APB2ENR  |= RCC_APB2ENR_TIM8EN;
// Full featured timers
    else if( timer == TIM2 ) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    else if( timer == TIM3 ) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    else if( timer == TIM4 ) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
    else if( timer == TIM5 ) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
// Basic timers
    else if( timer == TIM6 ) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;
    else if( timer == TIM7 ) RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
// Simple timers
    else if( timer == TIM15 ) RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    else if( timer == TIM16 ) RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    else if( timer == TIM17 ) RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
// Low Power timers
    else if( timer == (TIM_TypeDef *) LPTIM1 ) RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;
    else if( timer == (TIM_TypeDef *) LPTIM2 ) RCC->APB1ENR2 |= RCC_APB1ENR2_LPTIM2EN;

}


/**
 * @brief   Does timer support Quadrature
 *
 * Returns 1 if a timer supports quadrature encoder, 0 otherwise
 *
 */
static int Quadrature_SupportsQuadrature(TIM_TypeDef *timer) {

// Advanced timers
    if     ( timer == TIM1 ) return 1;
    else if( timer == TIM8 ) return 1;
// Full featured timers
    else if( timer == TIM2 ) return 1;
    else if( timer == TIM3 ) return 1;
    else if( timer == TIM4 ) return 1;
    else if( timer == TIM5 ) return 1;
// Low Power timers (not supported)
//    else if( timer == (TIM_TypeDef *) LPTIM1 ) return 1;
//
    else return 0;

}

/**
 * @brief   Enable clock for timers
 *
 * Returns 1 if a timer 32 bits, 0 otherwise
 *
 */
static inline int Quadrature_Is32(TIM_TypeDef *timer) {

    return (timer == TIM2) || (timer == TIM5);
}


/**
 * @brief  Configure pins for quadrature mode for a specific timer
 *
 * @note   void * used because there are two register layouts (TIM_TypeDef and LPTIM_TypeDef)
 */
static const TIM_PININFO *Quadrature_ConfigurePin(void *timer, int chn, int opt, const TIM_PININFO *pintable) {
const TIM_PININFO *p = pintable;
uint32_t m;
uint32_t pin;
uint32_t af;
GPIO_TypeDef *gpio;

    // search in table for timer and chn
    while( p->timer && p->timer != timer && p->chn != chn )
        p++;

    if( p->timer == 0 )
        return 0;

    // add opt to find the desired option
    p += opt;

    // Check

    if( p->timer != timer || p->chn != chn )
        return 0;

    gpio = p->gpio;
    pin  = p->pin;
    af   = p->af;

    m = 1<<pin;

    /* Configure pins as inputs */
    GPIO_Init(gpio,m,0);

    // Configuration for quadrature inputs
    GPIO_ConfigurePins(gpio,m,af);

    return p;
}


/**
 * @brief Initializes Quadrature Mode
 *
 * @note  It does not support LPTIMx!!!!
 */
void Quadrature_Init(TIM_TypeDef *timer, int cha, int chb, int mode) {

    // Continues only if there is support for quadrature
    if( !Quadrature_SupportsQuadrature(timer) )
        return;

    // Configure pins
    Quadrature_ConfigurePin(timer,1,cha,pintable);
    Quadrature_ConfigurePin(timer,2,chb,pintable);

    // Enable clock for timer
    Timer_EnableClock(timer);

    // Disable timer
    timer->CR1 &= ~TIM_CR1_CEN;

    // Set polarity
    timer->CCER &= ~(TIM_CCER_CC1P|TIM_CCER_CC2P);

    // Set filter
    timer->CCER &= ~(TIM_CCER_CC1NP|TIM_CCER_CC2NP);

    // Set upper counter limit
    if( Quadrature_Is32(timer) ) {
        timer->ARR = 0xFFFFFFFF;
    } else {
        timer->ARR = 0xFFFF;
    }

    //CC1S=01,CC2S=01

    // Set Encoder Interface by making SMS = 0b011
    // Alternatives are:
    //              0b001 count on T2 edges
    //              0b010 count on T1 edges
    //              0b011 count on both T1 and T2 edges
    switch(mode) {
    case QUADRATURE_MODE_X4:
        timer->SMCR = (timer->SMCR&~(TIM_SMCR_SMS_Msk))|(3<<TIM_SMCR_SMS_Pos);
        break;
    case QUADRATURE_MODE_X2_A:
        timer->SMCR = (timer->SMCR&~(TIM_SMCR_SMS_Msk))|(1<<TIM_SMCR_SMS_Pos);
        break;
    case QUADRATURE_MODE_X2_B:
        timer->SMCR = (timer->SMCR&~(TIM_SMCR_SMS_Msk))|(2<<TIM_SMCR_SMS_Pos);
        break;
    }

    // Enable it
    timer->CR1 |= TIM_CR1_CEN;

}

int32_t Quadrature_GetPosition(TIM_TypeDef *timer) {
uint32_t v;

    v = timer->CNT;

    if( Quadrature_Is32(timer) )
        v &= 0x7FFFFFFF;    // Clear UIF bit
    else
        v &= 0xFFFF;        // Clear high order bits

    return v;
}

void Quadrature_Load(TIM_TypeDef *timer, int32_t v) {


    if( Quadrature_Is32(timer) )
        v &= 0x7FFFFFFF;    // Clear UIF bit
    else
        v &= 0xFFFF;

    timer->CNT = v;

}

void Quadrature_Reset(TIM_TypeDef *timer) {

    timer->CNT = 0;
}


#ifdef QUADRATURE_LPTIMER
/**
 * @brief Initialize Quadrature mode on LPTIMER
 *
 * @note  Only LPTIM1 has Quadrature mode
 */
void Quadrature_LPInit(LPTIM_TypeDef *timer, int cha, int chb, int mode) {

    if( timer != LPTIM1 )
        return;

    // Timer disable
    timer->CR &= ~LPTIM_CR_ENABLE;

    // Configure pins
    Quadrature_ConfigurePin(timer,1,cha,lppintable);
    Quadrature_ConfigurePin(timer,2,chb,lppintable);

    // Continuous mode, internal clock, prescaler=1
    timer->CFGR &= ~(LPTIM_CFGR_COUNTMODE|LPTIM_CFGR_PRESC_Msk|LPTIM_CFGR_CKSEL_Msk);

    // Encoder mode
    timer->CFGR |= LPTIM_CFGR_ENC;


    // Enable clock for timer
    Timer_EnableClock( (TIM_TypeDef *)timer);

    // Count from 0 to 0xFFFF (16 bits)
    timer->ARR = 0xFFFF;

    // Couting mode (CKPOL)
    switch(mode) {
    case QUADRATURE_MODE_X4:
        timer->CFGR = (timer->CFGR&~LPTIM_CFGR_CKPOL_Msk)|(0<<LPTIM_CFGR_CKPOL_Pos);
        break;
    case QUADRATURE_MODE_X2_A:
        timer->CFGR = (timer->CFGR&~LPTIM_CFGR_CKPOL_Msk)|(1<<LPTIM_CFGR_CKPOL_Pos);
        break;
    case QUADRATURE_MODE_X2_B:
        timer->CFGR = (timer->CFGR&~LPTIM_CFGR_CKPOL_Msk)|(2<<LPTIM_CFGR_CKPOL_Pos);
        break;
    }


    // Timer enable
    timer->CR |= LPTIM_CR_ENABLE;

    // Timer start
    timer->CR |= LPTIM_CR_CNTSTRT;
}

/**
 * @brief  Get Position
 *
 * @note  Only LPTIM1 has Quadrature mode
 */
int32_t Quadrature_LPGetPosition(LPTIM_TypeDef *timer) {

    if( timer != LPTIM1 )
        return 0;

    return timer->CNT&0xFFFF;  // Should read twice?
}


/**
 * @brief Reset counter to zero
 *
 * @note  Only LPTIM1 has Quadrature mode
 */
void    Quadrature_LPReset(LPTIM_TypeDef *timer) {

    if( timer != LPTIM1 )
        return;

    timer->CNT = 0;
}

/**
 * @brief  Load counter with value v
 *
 * @note   Only LPTIM1 has Quadrature mode
 * @note   It is a 16 bit counter
 */
void    Quadrature_LPLoad(LPTIM_TypeDef *timer, int32_t v) {

    if( timer != LPTIM1 )
        return;

    timer->CNT = v&0xFFFF; // It is a 16 bit value
}

/**
 * @brief   Set Maximum Counter value
 *
 * @note
 */
void    Quadrature_LPSetMaximum(LPTIM_TypeDef *timer, int32_t v) {

    if( timer != LPTIM1 )
        return;

    timer->ARR = v&0xFFFF;

}

#endif
