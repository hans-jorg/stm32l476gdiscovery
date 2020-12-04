
/**
 * @file     main.c
 * @brief    Blink LEDs using counting delays
 * @version  V1.0
 * @date     31/10/2016
 * @version  V1.1
 * @date     22/04/2017
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
      volatile int x=700;
      while (x-- > 0)
         __NOP();
   }
}

/**
 * @brief The STM32L476 Discovery Board hat two LEDs.
 *
 * A red LED on Pin PE8 and a green LED on pin PB2
 *
 */

int main(void) {

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;   // Enable GPIO Port E
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIO Port B

    __DSB();

    /* Clear field and set desired value */
    GPIOB->MODER   = ((GPIOB->MODER&~(3<<(2*2)))    | (1<<(2*2)));   // Set to output
    GPIOB->OSPEEDR = ((GPIOB->OSPEEDR&~(3<<(2*2)))  | (3<<(2*2)));   // Set to high speed
    GPIOB->PUPDR   = ((GPIOB->PUPDR&~(3<<(2*2)))    | (1<<(2*2)));   // Set to pull up
    GPIOB->ODR    |= (1<<2);

    GPIOE->MODER   = ((GPIOE->MODER&~(3<<(8*2)))  | (1<<(8*2))); // Set to output
    GPIOE->OSPEEDR = ((GPIOE->OSPEEDR&~(3<<(8*2)))| (3<<(8*2))); // Set to high speed
    GPIOE->PUPDR   = ((GPIOE->PUPDR&~(3<<(8*2)))  | (1<<(8*2))); // Set to pull up
    GPIOE->ODR    &= ~(1<<8);



    for (;;) {
       ms_delay(500);
       GPIOB->ODR ^= (1 << 2);       // Use XOR to toggle output
       GPIOE->ODR ^= (1 << 8);     // Use XOR to toggle output
    }

    /* NEVER */
}
