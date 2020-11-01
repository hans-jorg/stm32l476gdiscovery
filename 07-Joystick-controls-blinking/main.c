
/**
 * @file     main.c
 * @brief    Blink LEDs controlled by joystick using CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library used
 *
 *
 ******************************************************************************/

/** @mainpage 05-joystick: Using joystick and LEDS
 *   @par Description: Blinks a LED using interrupt and CMSIS
 *   @note The green and red LED blink controlled by SysTickHandler
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
#define BITSET(V,M)                 ((V)|=(M))
#define BITCLEAR(V,M)               ((V)&=~(M))
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

#define JOY_ALL (JOY_DOWN|JOY_LEFT|JOY_UP|JOY_RIGHT)
//@}

/// semiperiod for red LED blinking
uint32_t perred = 1200;


/**
 * @brief SysTick Handler routine
 *
 * SysTick is configured to generate an interrupt every 1 ms.
 *
 * @note only red LED blinks
 */

void SysTick_Handler(void) {
static uint32_t cntred = 0;


    if( cntred == 0 ) {
        GPIOB->ODR ^= LED_RED;              // Use XOR to toggle output
        cntred = perred;
    } else {
        cntred--;
    }

}


/**
 * @brief Quick and dirty delay routine
 *
 * It gives approximately 1ms delay at 4 MHz (MSI)
 *
 */

//Quick hack, approximately 1ms delay
void ms_delay(volatile int ms) {
   while (ms-- > 0) {
      volatile int x=700;
      while (x-- > 0)
         __NOP();
   }
}


int main(void) {
uint32_t t;

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    SysTick_Config(SystemCoreClock/1000);

    t = RCC->AHB2ENR;
    BITSET(t,RCC_AHB2ENR_GPIOEEN);   // Enable GPIO Port E
    BITSET(t,RCC_AHB2ENR_GPIOBEN);   // Enable GPIO Port B
    BITSET(t,RCC_AHB2ENR_GPIOAEN);   // Enable GPIO Port A
    RCC->AHB2ENR = t;

    __DSB();

    /* Configure GPIOB */
    BITFIELDSET(GPIOB->MODER,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));         // Set to output
    BITFIELDSET(GPIOB->OSPEEDR,LED_RED_M2,BITVALUE(3,LED_RED_PIN));         // Set to high speed
    BITFIELDSET(GPIOB->PUPDR,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));         // Set to pull up
    BITSET(GPIOB->ODR,LED_RED);

    /* Configure GPIOE */
    BITFIELDSET(GPIOE->MODER,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));     // Set to output
    BITFIELDSET(GPIOE->OSPEEDR,LED_GREEN_M2,BITVALUE(3,LED_GREEN_PIN*2));   // Set to high speed
    BITFIELDSET(GPIOE->PUPDR,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));     // Set to pull up
    BITSET(GPIOE->ODR,LED_GREEN);

    /* Configure GPIOA */
    t = GPIOA->MODER;
    BITFIELDSET(t,BITVALUE(3,JOY_DOWN_PIN*2),0);                            // Set to input
    BITFIELDSET(t,BITVALUE(3,JOY_UP_PIN*2),0);                              // Set to input
    BITFIELDSET(t,BITVALUE(3,JOY_LEFT_PIN*2),0);                            // Set to input
    BITFIELDSET(t,BITVALUE(3,JOY_RIGHT_PIN*2),0);                           // Set to input
    GPIOA->MODER = t;
   // __DSB();

    t = GPIOA->PUPDR;
    BITFIELDSET(t,BITVALUE(3,JOY_DOWN_PIN*2),BITVALUE(2,JOY_DOWN_PIN*2));   // Set to pull down
    BITFIELDSET(t,BITVALUE(3,JOY_UP_PIN*2),BITVALUE(2,JOY_UP_PIN*2));       // Set to pull down
    BITFIELDSET(t,BITVALUE(3,JOY_LEFT_PIN*2),BITVALUE(2,JOY_LEFT_PIN*2));   // Set to pull down
    BITFIELDSET(t,BITVALUE(3,JOY_RIGHT_PIN*2),BITVALUE(2,JOY_RIGHT_PIN*2)); // Set to pull down
    GPIOA->PUPDR = t;
   // __DSB();

    uint32_t idrant = 0;
    for (;;) {
        uint32_t idr;
        idr = GPIOA->IDR;
        if( idr&JOY_DOWN ) {
            if( (idrant & JOY_DOWN) == 0 ) {
                if( perred>200 ) perred -= 200;
            }
            idrant |= JOY_DOWN;
        } else {
            idrant &= ~JOY_DOWN;
        }
        if( idr&JOY_UP ) {
            if( (idrant & JOY_UP) == 0 ) {
                if( perred<4000 ) perred += 300;
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
    }
}
