
/**
 * @file     joystick.c
 * @brief    Hardware Abstraction Layer (HAL) for joystick
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 *
 *
 ******************************************************************************/

#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "joystick.h"

/**
 * @brief Callback structure to store the pointer to functions
 */

static JoyStickCallBack callback = {
    .CenterButtonPressed = 0,
    .LeftButtonPressed   = 0,
    .RightButtonPressed  = 0,
    .UpButtonPressed     = 0,
    .DownButtonPressed   = 0
};


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
 * @brief Button interrupt routines
 *
 * @note EXTn catches interrupts on all GPIOs Pin n
 */
//@{

void EXTI1_IRQHandler(void) { /* LEFT BUTTON */
    if( (EXTI->IMR1&EXTI_IMR1_IM1) && (EXTI->PR1&EXTI_PR1_PIF1) ) {
        if( callback.LeftButtonPressed )
            callback.LeftButtonPressed();
        EXTI->PR1 |= EXTI_PR1_PIF1;
    }
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
}

void EXTI2_IRQHandler(void) { /* RIGHT BUTTON */
    if( (EXTI->IMR1&EXTI_IMR1_IM2) && (EXTI->PR1&EXTI_PR1_PIF2) ) {
        if( callback.RightButtonPressed )
            callback.RightButtonPressed();
        EXTI->PR1 |= EXTI_PR1_PIF2;
    }
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
}

void EXTI3_IRQHandler(void) { /* UP BUTTON */
    if( (EXTI->IMR1&EXTI_IMR1_IM3) && (EXTI->PR1&EXTI_PR1_PIF3) ) {
        if( callback.UpButtonPressed )
            callback.UpButtonPressed();
        EXTI->PR1 |= EXTI_PR1_PIF3;
    }
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
}

void EXTI9_5_IRQHandler(void) { /* DOWN BUTTON */
    if( (EXTI->IMR1&EXTI_IMR1_IM5) && (EXTI->PR1&EXTI_PR1_PIF5) ) {
        if( callback.DownButtonPressed )
            callback.DownButtonPressed();
        EXTI->PR1 |= EXTI_PR1_PIF5;
    }
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

void EXTI0_IRQHandler(void) { /* CENTER BUTTON */
    if( (EXTI->IMR1&EXTI_IMR1_IM0) && (EXTI->PR1&EXTI_PR1_PIF0) ) {
        if( callback.CenterButtonPressed )
            callback.CenterButtonPressed();
        EXTI->PR1 |= EXTI_PR1_PIF0;

    };
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
}

//@}

/**
 * @brief Joystick Initialization
 *
 * @note The only parameter is a pointer to a Callback structure filled
 *       with the pointer to functions, which will be called when the
 *       corresponding interrupt fires.
 */
uint32_t
JoyStick_Init(JoyStickCallBack *cb) {
uint32_t t;

    if( cb == 0 ) return 1;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable SysConfig, Comp, etc. */

    /* Initialize Callback Info */
    if( cb->CenterButtonPressed ) callback.CenterButtonPressed = cb->CenterButtonPressed;
    if( cb->LeftButtonPressed )   callback.LeftButtonPressed   = cb->LeftButtonPressed;
    if( cb->RightButtonPressed )  callback.RightButtonPressed  = cb->RightButtonPressed;
    if( cb->UpButtonPressed )     callback.UpButtonPressed     = cb->UpButtonPressed;
    if( cb->DownButtonPressed )   callback.DownButtonPressed   = cb->DownButtonPressed;

    t = RCC->AHB2ENR;
    t |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIO Port A
    RCC->AHB2ENR = t;

    /* Configure pins as input */
    t = GPIOA->MODER;
    SETBITFIELD(t,BITVALUE(3,JOY_DOWN_PIN*2),0);                            // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_UP_PIN*2),0);                              // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_LEFT_PIN*2),0);                            // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_RIGHT_PIN*2),0);                           // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_CENTER_PIN*2),0);                          // Set to input
    GPIOA->MODER = t;
   // __DSB();

    /* Configure pins as pull down */
    t = GPIOA->PUPDR;
    SETBITFIELD(t,BITVALUE(3,JOY_DOWN_PIN*2),BITVALUE(2,JOY_DOWN_PIN*2));   // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_UP_PIN*2),BITVALUE(2,JOY_UP_PIN*2));       // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_LEFT_PIN*2),BITVALUE(2,JOY_LEFT_PIN*2));   // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_RIGHT_PIN*2),BITVALUE(2,JOY_RIGHT_PIN*2)); // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_CENTER_PIN*2),BITVALUE(2,JOY_CENTER_PIN*2)); // Set to pull down
    GPIOA->PUPDR = t;
   // __DSB();

    /* Configure Interrupts */

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

    /* Enable Interrupts */
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    return 0;

}
