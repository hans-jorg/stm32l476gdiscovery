/**
 * @file     main.c
 * @brief    Blink LEDs using counting delays and CMSIS (Heavy use of macros)
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     The blinking frequency depends on core frequency
 * @note     Direct access to registers
 * @note     No library used
 *
 *
 ******************************************************************************/


#include "stm32l476xx.h"
#include "system_stm32l476.h"

/**
 * @brief Quick and dirty delay routine
 *
 * It gives approximately 1ms delay at 4 MHz (MSI)
 *
 */

void ms_delay(volatile int ms) {
   while (ms-- > 0) {
      volatile int x=14000;
      while (x-- > 0)
         __NOP();
   }
}

/**
 * @brief Macros for bit and bitfields manipulation
 */

///@{
#define BIT(N)                     (1UL<<(N))
#define BITMASK(M,N)               ((BIT((M)-(N)+1)-1)<<(N))
#define BITVALUE(V,N)              ((V)<<(N))
#define BITSET(V,M)                (V)|=(M)
#define BITCLEAR(V,M)              (V)&=~(M)
#define BITTOGGLE(V,M)             (V)^=(M)
#define BITFIELDSET(VAR,MASK,VAL)  (VAR) = ((VAR)&~(MASK))|(VAL)
#define BITFIELDCLEAR(VAR,MASK)    (VAR) &= ~(MASK)
///@}


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

int main(void) {

    SystemCoreClockSet(MSI48M_CLOCKSRC, 0, 3, 0);

    BITSET(RCC->AHB2ENR,RCC_AHB2ENR_GPIOEEN);   // Enable GPIO Port E
    BITSET(RCC->AHB2ENR,RCC_AHB2ENR_GPIOBEN);   // Enable GPIO Port B

    __DSB();

    /* Clear field and set desired value */
    BITFIELDSET(GPIOB->MODER,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));       // Set to output
    BITFIELDSET(GPIOB->OSPEEDR,LED_RED_M2,BITVALUE(3,LED_RED_PIN));       // Set to high speed
    BITFIELDSET(GPIOB->PUPDR,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));       // Set to pull up
    BITSET(GPIOB->ODR,LED_RED);

    BITFIELDSET(GPIOE->MODER,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));   // Set to output
    BITFIELDSET(GPIOE->OSPEEDR,LED_GREEN_M2,BITVALUE(3,LED_GREEN_PIN*2)); // Set to high speed
    BITFIELDSET(GPIOE->PUPDR,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));   // Set to pull up
    BITCLEAR(GPIOE->ODR,LED_GREEN);



    for (;;) {
       ms_delay(500);
       BITTOGGLE(GPIOB->ODR,LED_RED);       // Use XOR to toggle output
       BITTOGGLE(GPIOE->ODR,LED_GREEN);     // Use XOR to toggle output
    }
}
