/**
 * @file     main.c
 * @brief    Use joystick buttons to control blinking
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     An abstraction layer for LEDs is used (led.[ch])
 *
 ******************************************************************************/

/** @mainpage 09-blink-enhanced: Using joystick and LEDS
 *   @par Description: Uses joystick buttons to control LEDs.
 *   @par An Hardware Abstraction Layer for LEDs is used.
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"

/**
 * @brief Macros for bit and bitfields manipulation
 */

///@{
#define BIT(N)                      (1UL<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)               ((V)<<(N))
#define SETBITFIELD(VAR,MASK,VAL)   (VAR) = ((VAR)&~(MASK))|(VAL)
//@}



/**
 * @brief Joystick Symbols
 * @note Joystick on GPIO Port A
 */

//@{
#define JOY_CENTER_PIN  (0)
#define JOY_DOWN_PIN    (5)
#define JOY_LEFT_PIN    (1)
#define JOY_RIGHT_PIN   (2)
#define JOY_UP_PIN      (3)

#define JOY_DOWN         BIT(JOY_DOWN_PIN)
#define JOY_LEFT         BIT(JOY_LEFT_PIN)
#define JOY_UP           BIT(JOY_UP_PIN)
#define JOY_RIGHT        BIT(JOY_RIGHT_PIN)
#define JOY_CENTER       BIT(JOY_CENTER_PIN)

#define JOY_ALL (JOY_DOWN|JOY_LEFT|JOY_UP|JOY_RIGHT|JOY_CENTER)
//@}

/**
 * @brief Control blinking period
 */

//@{
uint32_t semiperiod_max   = 5000;
uint32_t semiperiod_min   = 100;
uint32_t semiperiod_step  = 100;

uint32_t semiperiod_red   = 500;        // 1 sec
uint32_t semiperiod_green = 750;        // 1.5 sec
static uint32_t cnt_red = 0;
static uint32_t cnt_green = 0;
//@}

/// Blinking state (ative|inactive)
uint32_t blinking = 1;                  // blinking active

/// Debounce counter (only for CENTER, no neeed for others)
uint32_t debounce_counter = 0;

/**
 * @brief SysTick Handler
 *
 * @note Almost everything happens here
 */

void SysTick_Handler(void) {

    if( debounce_counter ) debounce_counter--;

    if( blinking ) {
        if( cnt_red == 0 ) {
            LED_Toggle(LED_RED);
            cnt_red = semiperiod_red;
        }
        cnt_red--;

        if( cnt_green == 0 ) {
            LED_Toggle(LED_GREEN);
            cnt_green = semiperiod_green;
        }
        cnt_green--;
    }
}


/**
 * @brief Button interrupt routines
 *
 * @note EXTn catches interrupts on all GPIOs Pin n
 */
//@{

void EXTI0_IRQHandler(void) { /* CENTER PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM0) && (EXTI->PR1&EXTI_PR1_PIF0) ) {
        if( debounce_counter == 0 ) {
            if( blinking ) {
                /* Turn off all LEDS */
                LED_Write(0,LED_ALL);
                blinking = 0;
            } else {
                blinking = 1;
            }
            debounce_counter = 40;
        }
        EXTI->PR1 |= EXTI_PR1_PIF0;
        cnt_red = 0;
        cnt_green = 0;
    };
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
}

void EXTI1_IRQHandler(void) { /* LEFT PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM1) && (EXTI->PR1&EXTI_PR1_PIF1) ) {
        /* Decrease period of green LED blinking  */
        if( semiperiod_green > (semiperiod_min+semiperiod_step) )
            semiperiod_green -= semiperiod_step;
        EXTI->PR1 |= EXTI_PR1_PIF1;
    }
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void EXTI2_IRQHandler(void) { /* RIGHT PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM2) && (EXTI->PR1&EXTI_PR1_PIF2) ) {
        /* Increase period of green LED blinking  */
        if( semiperiod_green < (semiperiod_max-semiperiod_step) )
            semiperiod_green += semiperiod_step;
        EXTI->PR1 |= EXTI_PR1_PIF2;
    }
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
}

void EXTI3_IRQHandler(void) { /* UP PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM3) && (EXTI->PR1&EXTI_PR1_PIF3) ) {
        /* Decrease period of red LED blinking  */
        if( semiperiod_red > (semiperiod_min+semiperiod_step) )
            semiperiod_red -= semiperiod_step;
        EXTI->PR1 |= EXTI_PR1_PIF3;
    }
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
}

void EXTI9_5_IRQHandler(void) { /* DOWN PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM5) && (EXTI->PR1&EXTI_PR1_PIF5) ) {
        /* Increase period of red LED blinking  */
        if( semiperiod_red < (semiperiod_max-semiperiod_step) )
            semiperiod_red += semiperiod_step;
        EXTI->PR1 |= EXTI_PR1_PIF5;
    }
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}


//@}


int main(void) {
uint32_t t;

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable SysConfig, Comp, etc. */

    LED_Init(LED_ALL);

    t = RCC->AHB2ENR;
    t |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIO Port A
    RCC->AHB2ENR = t;

    t = GPIOA->MODER;
    SETBITFIELD(t,BITVALUE(3,JOY_DOWN_PIN*2),0);                            // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_UP_PIN*2),0);                              // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_LEFT_PIN*2),0);                            // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_RIGHT_PIN*2),0);                           // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_CENTER_PIN*2),0);                          // Set to input
    GPIOA->MODER = t;
   // __DSB();

    t = GPIOA->PUPDR;
    SETBITFIELD(t,BITVALUE(3,JOY_DOWN_PIN*2),BITVALUE(2,JOY_DOWN_PIN*2));   // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_UP_PIN*2),BITVALUE(2,JOY_UP_PIN*2));       // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_LEFT_PIN*2),BITVALUE(2,JOY_LEFT_PIN*2));   // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_RIGHT_PIN*2),BITVALUE(2,JOY_RIGHT_PIN*2)); // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_CENTER_PIN*2),BITVALUE(2,JOY_CENTER_PIN*2)); // Set to pull down
    GPIOA->PUPDR = t;
   // __DSB();

    /* Enable interrupts */

    EXTI->IMR1  |=  (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5));
    EXTI->RTSR1 |=  (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5));
    EXTI->FTSR1 &= ~(BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5));

    t = SYSCFG->EXTICR[0];
    SETBITFIELD(t,BITVALUE(7,0),0);  /* EXTI0 : Center Pin */
    SETBITFIELD(t,BITVALUE(7,4),0);  /* EXTI1 : Left Pin */
    SETBITFIELD(t,BITVALUE(7,8),0);  /* EXTI2 : Right Pin */
    SETBITFIELD(t,BITVALUE(7,12),0); /* EXTI3 : Up Pin */
    SYSCFG->EXTICR[0] = t;

    t = SYSCFG->EXTICR[1];
    SETBITFIELD(t,BITVALUE(7,4),0);  /* EXTI5 : Down Pin */
    SYSCFG->EXTICR[1] = t;

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    for (;;) {}
}
