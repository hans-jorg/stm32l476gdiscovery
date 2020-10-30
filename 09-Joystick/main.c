/**
 * @file     main.c
 * @brief    Use joystick buttons to control LEDs
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 *
 *
 ******************************************************************************/

/** @mainpage 07-joystick: Using joystick and LEDS
 *   @par Description: Uses joystick buttons to control LEDs
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"

/**
 * @brief Macros for bit and bitfields manipulation
 */

///@{
#define BIT(N)                      (1UL<<(N))
#define BITMASK(M,N)                ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)               ((V)<<(N))
#define BITSET(V,M)                 (V)|=(M)
#define BITCLEAR(V,M)               (V)&=~(M)
#define BITFIELDSET(VAR,MASK,VAL)   (VAR) = ((VAR)&~(MASK))|(VAL)
//@}

/**
 * @brief LED Symbols
 */

//@{
// LED on GPIO Port E
#define LED_GREEN_PIN  (8)
// LED on GPIO Port B
#define LED_RED_PIN    (2)

#define LED_GREEN    BIT(LED_GREEN_PIN)
#define LED_RED      BIT(LED_RED_PIN)

#define LED_GREEN_M2 BITVALUE(3,LED_GREEN_PIN*2)
#define LED_RED_M2   BITVALUE(3,LED_RED_PIN*2)
//@}

/**
 * @brief Joystick Symbols
 */

//@{
// Joystick on GPIO Port A
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
 * @brief This interrupt is triggered when there is no pending interrupts
 */

void PendSV(void) {

}

/**
 * @brief SysTick handler (not used)
 */

void SysTick_Handler(void) {

}

/**
 * @brief Interrupt routines for Joystick pins
 */

//@{
void EXTI0_IRQHandler(void) { /* CENTER PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM0) && (EXTI->PR1&EXTI_PR1_PIF0) ) {
        /* Turn off all LEDS */
        GPIOE->ODR &= ~LED_GREEN;
        GPIOB->ODR &= ~LED_RED;
        EXTI->PR1 |= EXTI_PR1_PIF0;
    };
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
}

void EXTI1_IRQHandler(void) { /* LEFT PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM1) && (EXTI->PR1&EXTI_PR1_PIF1) ) {
        /*Turn off green LED */
        GPIOE->ODR &= ~LED_GREEN;
        EXTI->PR1 |= EXTI_PR1_PIF1;
    }
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void EXTI2_IRQHandler(void) { /* RIGHT PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM2) && (EXTI->PR1&EXTI_PR1_PIF2) ) {
        /* Turn on green LED */
        GPIOE->ODR |= LED_GREEN;
        EXTI->PR1 |= EXTI_PR1_PIF2;
    }
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
}

void EXTI3_IRQHandler(void) { /* UP PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM3) && (EXTI->PR1&EXTI_PR1_PIF3) ) {
        /* Turn on red LED */
        GPIOB->ODR |= LED_RED;
        EXTI->PR1 |= EXTI_PR1_PIF3;
    }
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
}

void EXTI9_5_IRQHandler(void) { /* DOWN PIN */
    if( (EXTI->IMR1&EXTI_IMR1_IM5) && (EXTI->PR1&EXTI_PR1_PIF5) ) {
        /* Turn off red LED */
        GPIOB->ODR &= ~LED_RED;
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

    t = RCC->AHB2ENR;
    t |= RCC_AHB2ENR_GPIOEEN;   // Enable GPIO Port E
    t |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIO Port B
    t |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIO Port A
    RCC->AHB2ENR = t;

    __DSB();

    /* Clear field and set desired value */
    BITFIELDSET(GPIOB->MODER,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));         // Set to output
    BITFIELDSET(GPIOB->OSPEEDR,LED_RED_M2,BITVALUE(3,LED_RED_PIN));         // Set to high speed
    BITFIELDSET(GPIOB->PUPDR,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));         // Set to pull up
    GPIOB->ODR |= LED_RED;

    BITFIELDSET(GPIOE->MODER,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));     // Set to output
    BITFIELDSET(GPIOE->OSPEEDR,LED_GREEN_M2,BITVALUE(3,LED_GREEN_PIN*2));   // Set to high speed
    BITFIELDSET(GPIOE->PUPDR,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));     // Set to pull up
    GPIOE->ODR  |= LED_GREEN;

    t = GPIOA->MODER;
    BITFIELDSET(t,BITVALUE(3,JOY_DOWN_PIN*2),0);                            // Set to input
    BITFIELDSET(t,BITVALUE(3,JOY_UP_PIN*2),0);                              // Set to input
    BITFIELDSET(t,BITVALUE(3,JOY_LEFT_PIN*2),0);                            // Set to input
    BITFIELDSET(t,BITVALUE(3,JOY_RIGHT_PIN*2),0);                           // Set to input
    BITFIELDSET(t,BITVALUE(3,JOY_CENTER_PIN*2),0);                          // Set to input
    GPIOA->MODER = t;
   // __DSB();

    t = GPIOA->PUPDR;
    BITFIELDSET(t,BITVALUE(3,JOY_DOWN_PIN*2),BITVALUE(2,JOY_DOWN_PIN*2));   // Set to pull down
    BITFIELDSET(t,BITVALUE(3,JOY_UP_PIN*2),BITVALUE(2,JOY_UP_PIN*2));       // Set to pull down
    BITFIELDSET(t,BITVALUE(3,JOY_LEFT_PIN*2),BITVALUE(2,JOY_LEFT_PIN*2));   // Set to pull down
    BITFIELDSET(t,BITVALUE(3,JOY_RIGHT_PIN*2),BITVALUE(2,JOY_RIGHT_PIN*2)); // Set to pull down
    BITFIELDSET(t,BITVALUE(3,JOY_CENTER_PIN*2),BITVALUE(2,JOY_CENTER_PIN*2)); // Set to pull down
    GPIOA->PUPDR = t;
   // __DSB();

    /* Enable interrupts */

    EXTI->IMR1  |=  (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5));
    EXTI->RTSR1 |=  (BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5));
//    EXTI->FTSR1 &= ~(BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(5));

    t = SYSCFG->EXTICR[0];
    BITFIELDSET(t,BITVALUE(7,0),0);  /* EXTI0 : Center Pin */
    BITFIELDSET(t,BITVALUE(7,4),0);  /* EXTI1 : Left Pin */
    BITFIELDSET(t,BITVALUE(7,8),0);  /* EXTI2 : Right Pin */
    BITFIELDSET(t,BITVALUE(7,12),0); /* EXTI3 : Up Pin */
    SYSCFG->EXTICR[0] = t;

    t = SYSCFG->EXTICR[1];
    BITFIELDSET(t,BITVALUE(7,4),0);  /* EXTI5 : Down Pin */
    SYSCFG->EXTICR[1] = t;

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);


    for (;;) {}
}
