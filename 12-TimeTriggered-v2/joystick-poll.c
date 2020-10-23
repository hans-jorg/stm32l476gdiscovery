

/**
 * @file     joystick.c
 * @brief    Hardware Abstraction Layer (HAL) for joystick using polling
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     No interrupts!!!
 *
 *
 ******************************************************************************/

#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "joystick.h"


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
 * @brief Button Initialization
 *
 * @note No parameters.
 */
uint32_t
JoyStick_Init(void) {
uint32_t t;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable SysConfig, Comp, etc. */

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

    return 0;

}

/**
 * @brief Joystick Read
 *
 * @note Returns the current state of joystick buttons.
 * @note Use JOY_DOWN, JOY_LEFT, JOY_UP, JOY_RIGHT, JOY_CENTER bit masks
 *       to verify if a specific button is pressed
 */

uint32_t
JoyStick_Read(void) {
uint32_t v;

    v = GPIOA->IDR;
    v &= JOY_ALL;       // Clear bit not related to joystick

    return v;
}
