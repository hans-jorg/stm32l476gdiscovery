
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

/** @mainpage 06-joystick: Using joystick and LEDS
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
#define SETBITFIELD(VAR,MASK,VAL)   (VAR) = ((VAR)&~(MASK))|(VAL)
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


int main(void) {
uint32_t t;

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    t = RCC->AHB2ENR;
    t |= RCC_AHB2ENR_GPIOEEN;   // Enable GPIO Port E
    t |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIO Port B
    t |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIO Port A
    RCC->AHB2ENR = t;

    __DSB();

    /* Clear field and set desired value */
    SETBITFIELD(GPIOB->MODER,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));          // Set to output
    SETBITFIELD(GPIOB->OSPEEDR,LED_RED_M2,BITVALUE(3,LED_RED_PIN));          // Set to high speed
    SETBITFIELD(GPIOB->PUPDR,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));          // Set to pull up
    GPIOB->ODR |= LED_RED;

    SETBITFIELD(GPIOE->MODER,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));      // Set to output
    SETBITFIELD(GPIOE->OSPEEDR,LED_GREEN_M2,BITVALUE(3,LED_GREEN_PIN*2));    // Set to high speed
    SETBITFIELD(GPIOE->PUPDR,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));      // Set to pull up
    GPIOE->ODR  |= LED_GREEN;

    t = GPIOA->MODER;
    SETBITFIELD(t,BITVALUE(3,JOY_DOWN_PIN*2),0);                             // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_UP_PIN*2),0);                               // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_LEFT_PIN*2),0);                             // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_RIGHT_PIN*2),0);                            // Set to input
    SETBITFIELD(t,BITVALUE(3,JOY_CENTER_PIN*2),0);                           // Set to input
    GPIOA->MODER = t;
   // __DSB();

    t = GPIOA->PUPDR;
    SETBITFIELD(t,BITVALUE(3,JOY_DOWN_PIN*2),BITVALUE(2,JOY_DOWN_PIN*2));    // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_UP_PIN*2),BITVALUE(2,JOY_UP_PIN*2));        // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_LEFT_PIN*2),BITVALUE(2,JOY_LEFT_PIN*2));    // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_RIGHT_PIN*2),BITVALUE(2,JOY_RIGHT_PIN*2));  // Set to pull down
    SETBITFIELD(t,BITVALUE(3,JOY_CENTER_PIN*2),BITVALUE(2,JOY_CENTER_PIN*2));// Set to pull down
    GPIOA->PUPDR = t;
   // __DSB();

    uint32_t idrant = 0;
    for (;;) {
        uint32_t idr;
        idr = GPIOA->IDR;
        if( idr&JOY_DOWN ) {
            if( (idrant & JOY_DOWN) == 0 ) {
                GPIOB->ODR &= ~LED_RED;
            }
            idrant |= JOY_DOWN;
        } else {
            idrant &= ~JOY_DOWN;
        }
        if( idr&JOY_UP ) {
            if( (idrant & JOY_UP) == 0 ) {
                GPIOB->ODR |= LED_RED;
            }
            idrant |= JOY_UP;
        } else {
            idrant &= ~JOY_UP;
        }

        if( idr&JOY_LEFT ) {
            if( (idrant & JOY_LEFT) == 0 ) {
                GPIOE->ODR &= ~LED_GREEN;
            }
            idrant |= JOY_LEFT;
        } else {
            idrant &= ~JOY_LEFT;
        }
        if( idr&JOY_RIGHT ) {
            if( (idrant & JOY_RIGHT) == 0 ) {
                GPIOE->ODR |= LED_GREEN;
            }
            idrant |= JOY_RIGHT;
        } else {
            idrant &= ~JOY_RIGHT;
        }

        if( idr&JOY_CENTER ) {
            if( (idrant & JOY_CENTER) == 0 ) {
                GPIOE->ODR &= ~LED_GREEN;
                GPIOB->ODR &= ~LED_RED;
            }
            idrant |= JOY_CENTER;
        } else {
            idrant &= ~JOY_CENTER;
        }


    }
}
