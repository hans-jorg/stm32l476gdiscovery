/**
 * @file    pwm.c
 * @brief   HAL for PWM generation
 *
 * @version 1.0
 *
 */


#include  <stdint.h>
#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "gpio2.h"
#include "pwm.h"

/*
 * The following timers have support for PWM
 *    * Advanced control timers (TIM1/TIM8)
 *    * General purpose timers (TIM2/TIM3/TIM4/TIM5)
 *    * General purpose timers (TIM14/TIM16/TIM17)
 *    * Low power timer (LPTIM)
 *
 * The counter in a timer is configured to count upwards until a
 *   specified value and then automatically reload
 * A reference value is set and is compared with the counter. If the counter
 *   lower than the reference, the output is high. If it is higher, the output
 *   is low.
 *
 * Since there are four reference registers, there are four output compare pins
 *
 *  On the STM32L476 Discovery Board most of these pins are already used.
 *  But, PB6 and PB7 are on pins 15 and 16 of header P1.
 *  They are labeled I2C1_SCL and I2C1_SDA.
 *  Others will conflick with LED or Joystick.
 */


typedef struct {
    TIM_TypeDef     *timer;
    uint16_t        channel;
    GPIO_TypeDef    *gpio;
    uint16_t         pin;
    uint16_t         af;
} TIM_PinInfo;

static const TIM_PinInfo pininfo[] = {
/*    timer ch gpio   pin   af  */
  {   TIM1, 1, GPIOA,   8,   1   },
  {   TIM1, 2, GPIOA,   9,   1   },
  {   TIM1, 3, GPIOA,  10,   1   },
  {   TIM1, 4, GPIOA,  11,   1   },
  {   TIM1, 1, GPIOE,   9,   1   },
  {   TIM1, 2, GPIOE,  11,   1   },
  {   TIM1, 3, GPIOE,  13,   1   },
  {   TIM1, 4, GPIOE,  14,   1   },

  {   TIM2, 1, GPIOA,   0,   1   },
  {   TIM2, 1, GPIOA,   5,   1   },
  {   TIM2, 1, GPIOA,  15,   1   },
  {   TIM2, 2, GPIOA,   1,   1   },
  {   TIM2, 2, GPIOB,   3,   1   },
  {   TIM2, 3, GPIOA,   2,   1   },
  {   TIM2, 3, GPIOB,  10,   1   },
  {   TIM2, 4, GPIOA,   3,   1   },
  {   TIM2, 4, GPIOB,  11,   1   },

  {   TIM3, 1, GPIOA,   6,   2   },
  {   TIM3, 1, GPIOB,   4,   2   },
  {   TIM3, 1, GPIOC,   6,   2   },
  {   TIM3, 1, GPIOE,   3,   2   },
  {   TIM3, 2, GPIOA,   7,   2   },
  {   TIM3, 2, GPIOB,   5,   2   },
  {   TIM3, 2, GPIOC,   7,   2   },
  {   TIM3, 2, GPIOE,   4,   2   },
  {   TIM3, 3, GPIOB,   0,   2   },
  {   TIM3, 3, GPIOC,   8,   2   },
  {   TIM3, 3, GPIOE,   5,   2   },
  {   TIM3, 4, GPIOB,   1,   2   },
  {   TIM3, 4, GPIOC,   9,   2   },
  {   TIM3, 4, GPIOE,   6,   2   },

  {   TIM4, 1, GPIOB,   6,   2   },
  {   TIM4, 1, GPIOD,  12,   2   },
  {   TIM4, 2, GPIOD,   7,   2   },
  {   TIM4, 2, GPIOD,  12,   2   },
  {   TIM4, 3, GPIOB,   8,   2   },
  {   TIM4, 3, GPIOD,  14,   2   },
  {   TIM4, 4, GPIOB,   9,   2   },
  {   TIM4, 4, GPIOD,  15,   2   },

  {   TIM5, 1, GPIOA,   0,   2   },
  {   TIM5, 1, GPIOF,   6,   2   },
  {   TIM5, 2, GPIOA,   1,   2   },
  {   TIM5, 2, GPIOF,   7,   2   },
  {   TIM5, 3, GPIOA,   2,   2   },
  {   TIM5, 3, GPIOF,   8,   2   },
  {   TIM5, 4, GPIOA,   3,   2   },
  {   TIM5, 4, GPIOF,   9,   2   },


  {   TIM8, 1, GPIOC,   6,   3   },
  {   TIM8, 2, GPIOC,   7,   3   },
  {   TIM8, 3, GPIOC,   8,   3   },
  {   TIM8, 4, GPIOC,   9,   3   },
  {      0, 0, 0,       0,   0,  }
};

#define TIMER_TYPEADVANCED              1
#define TIMER_TYPEGENERAL               2
#define TIMER_TYPESIMPLE                3
#define TIMER_TYPEBASIC                 4
#define TIMER_TYPELOWPOWER              5

#define TIMER_SIZE16                    0
#define TIMER_SIZE32                    1

typedef struct {
    TIM_TypeDef     *timer;
    uint32_t        clockenable;
    unsigned        apb;
    unsigned        id;
    unsigned        size;
    unsigned        type;
} TIM_Parameters;

static const TIM_Parameters timparameters[] = {
    {   TIM1,    RCC_APB2ENR_TIM1EN,    2,  1, TIMER_SIZE16, TIMER_TYPEADVANCED },
    {   TIM2,    RCC_APB1ENR1_TIM2EN,   1,  2, TIMER_SIZE32, TIMER_TYPEGENERAL },
    {   TIM3,    RCC_APB1ENR1_TIM3EN,   1,  3, TIMER_SIZE16, TIMER_TYPEGENERAL },
    {   TIM4,    RCC_APB1ENR1_TIM4EN,   1,  4, TIMER_SIZE16, TIMER_TYPEGENERAL },
    {   TIM5,    RCC_APB1ENR1_TIM5EN,   1,  5, TIMER_SIZE32, TIMER_TYPEGENERAL },
    {   TIM6,    RCC_APB1ENR1_TIM6EN,   1,  6, TIMER_SIZE16, TIMER_TYPEBASIC },
    {   TIM7,    RCC_APB1ENR1_TIM7EN,   1,  7, TIMER_SIZE16, TIMER_TYPEBASIC },
    {   TIM8,    RCC_APB2ENR_TIM8EN,    2,  8, TIMER_SIZE16, TIMER_TYPEADVANCED },
    {   TIM15,   RCC_APB2ENR_TIM15EN,   2, 15, TIMER_SIZE16, TIMER_TYPEGENERALSIMPLE },
    {   TIM16,   RCC_APB2ENR_TIM16EN,   2, 16, TIMER_SIZE16, TIMER_TYPEGENERALSIMPLE },
    {   TIM17,   RCC_APB2ENR_TIM17EN,   2, 17, TIMER_SIZE16, TIMER_TYPEGENERALSIMPLE },
    {   LPTIM1,  RCC_APB1ENR2_LPTIM2EN, 3, 20, TIMER_SIZE16, TIMER_TYPELOWPOWER }
    {   LPTIM2,  RCC_APB1ENR2_LPTIM2EN, 3, 20, TIMER_SIZE16, TIMER_TYPELOWPOWER }
    {   0,  0   }
};

static const TIM_Parameters *findparameters(TIM_TypeDef *timer) {
const TIM_Parameters *p;


    p = timparameters;

    while( p->timer&&(p->timer==timer)) {p++;}

    if( p->timer )
        return p;
    else
        return 0;
}

static const TIM_PinInfo *findpininformation(TIM_TypeDef *timer,
                     unsigned channel,
                     GPIO_TypeDef *gpio,
                     unsigned pin) {
const TIM_PinInfo *p;

    p = pininfo;
    while( p->timer ) {
        if( (p->timer == timer)
            && (p->channel==channel)
            && (p->gpio == gpio)
            && (p->pin == pin) ) {
            return p;
        }
        p++;
    }

    return 0;
}


static void GPIO_ConfigureAlternateFunction(GPIO_TypeDef *gpio, unsigned pin, unsigned af) {
int pos;

    GPIO_EnableClock(gpio);
    if(pin < 8 ) {
        pos = pin*4;
        gpio->AFR[0] = (gpio->AFR[0]&~(0xF<<pos))|(af<<pos);
    } else {
        pos = (pin-8)*4;
        gpio->AFR[1] = (gpio->AFR[1]&~(0xF<<pos))|(af<<pos);
    }

}

int PWM_Init(TIM_TypeDef *timer, unsigned channel, GPIO_TypeDef *gpio, unsigned pin) {
const TIM_Parameters *ptim;
const TIM_PinInfo *pinfo;
int af;

    ptim = findparameters(timer);
    if( !ptim )
        return -1;

    pinfo = findpininformation(timer,channel,gpio,pin);
    if( !pinfo )
        return -2;

    // Disable timer (Just in case)
    timer->CR1 &= ~TIM_CR1_CEN;
    /*
     * Configure GPIO
     */
    /* Configure pin as output and set alternate function */
    GPIO_EnableClock(gpio);
    GPIO_ConfigurePins(gpio, (1U<<pin), GPIO_CONF_OUT);
    GPIO_ConfigureAlternateFunction(gpio,pin,pinfo->af);


    /*
     * Configure timer
     */

    /* Enable clock */
    if( ptim->apb == 1 ) {
        RCC->APB1ENR1 |=  ptim->clockenable;
    } else {
        RCC->APB2ENR |=  ptim->clockenable;
    }
    /* Set divider to 1, i.e. prescaler to 0 */
    timer->PSC = 0;
    timer->SMCR = (timer->SMCR&~(TIM_SMCR_SMS_Msk))|(0<<TIM_SMCR_SMS_Pos);

    switch(channel) {
    case 1:
        timer->CCR1 = 0;
        timer->CCMR = 0; // OCxM=110|111 ???? OCxPE=1?
        break;
    case 2:
        timer->CCR2 = 0;

        break;
    case 3:
        timer->CCR3 = 0;

        break;
    case 4:
        timer->CCR4 = 0;

        break;
    default:
        return -3;
    }

    timer->ARR  = (uint32_t) (-1);



    // timer enable
    timer->CR1 &= ~TIM_CR1_DIR;
    timer->CR1 |= TIM_CR1_ARPE|TIM_CR1_CEN;

    return 0;
}

int PWM_Set(TIM_TypeDef *timer, unsigned channel, unsigned value) {

    switch( channel ) {
    case 1:
        timer->CCR1 = value;
        break;
    case 2:
        timer->CCR2 = value;
        break;
    case 3:
        timer->CCR3 = value;
        break;
    case 4:
        timer->CCR4 = value;
        break;
    default:
        return -1;
    }
    return 0;
}

int PWM_Config(TIM_TypeDef *timer, unsigned channel, unsigned top, unsigned div, int pol) {

    if( pol ) {
        timer->CCER = 0; // CCxP  | CCxE
    } else {
        timer->CCER = 0; // CCxP  | CCxE
    }
    return 0;
}

int PWM_StopTImer(TIM_TypeDef *timer) {

    timer->CR1 &= ~TIM_CR1_CEN;
    return 0;
}

int PWM_StartTImer(TIM_TypeDef *timer) {

    timer->CR1 |= TIM_CR1_CEN;
    return 0;
}

