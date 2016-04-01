
/**
 * @file     led.c
 * @brief    Hardware Abstraction Layer (HAL) for LEDs
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
 * @brief LED Symbols for configuration fields
 */
//@{
#define LED_GREEN_M2 BITVALUE(3,LED_GREEN_PIN*2)
#define LED_RED_M2   BITVALUE(3,LED_RED_PIN*2)
//@}

/**
 * @brief LED Init
 *
 * @note Initializes specified LEDs
 */

uint32_t
LED_Init(uint32_t leds) {
uint32_t t;

    t = RCC->AHB2ENR;
    if( leds&LED_GREEN ) t |= RCC_AHB2ENR_GPIOEEN;   // Enable GPIO Port E
    if( leds&LED_RED   ) t |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIO Port B
    RCC->AHB2ENR = t;

    __DSB();

    /* Clear field and set desired value */
    if( leds&LED_RED ) {
        SETBITFIELD(GPIOB->MODER,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));         // Set to output
        SETBITFIELD(GPIOB->OSPEEDR,LED_RED_M2,BITVALUE(3,LED_RED_PIN));         // Set to high speed
        SETBITFIELD(GPIOB->PUPDR,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));         // Set to pull up
        GPIOB->ODR |= LED_RED;
    }

    if( leds&LED_GREEN ) {
        SETBITFIELD(GPIOE->MODER,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));     // Set to output
        SETBITFIELD(GPIOE->OSPEEDR,LED_GREEN_M2,BITVALUE(3,LED_GREEN_PIN*2));   // Set to high speed
        SETBITFIELD(GPIOE->PUPDR,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));     // Set to pull up
        GPIOE->ODR  |= LED_GREEN;
    }

    return 0;
}

/**
 * @brief LED Write
 *
 * @note Turns ON selected LEDs (parameter on) and
 *       then turn OFF the selected LEDs (parameter off)
 */

uint32_t
LED_Write(uint32_t on, uint32_t off) {

    // Turn selected leds on
    if( on&LED_RED)    GPIOB->ODR |= LED_RED;
    if( on&LED_GREEN ) GPIOE->ODR |= LED_GREEN;

    // Turn selected leds off
    if( off&LED_RED)    GPIOB->ODR &= ~LED_RED;
    if( off&LED_GREEN ) GPIOE->ODR &= ~LED_GREEN;

    return 0;
}

/**
 * @brief LED Toggle
 *
 * @note Toggles only the specificied LEDs
 */

uint32_t
LED_Toggle(uint32_t leds) {

    if( leds&LED_RED   ) GPIOB->ODR ^= LED_RED;

    if( leds&LED_GREEN ) GPIOE->ODR ^= LED_GREEN;


    return 0;
}
