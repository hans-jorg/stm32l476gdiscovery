
/**
 * @file     main.c
 * @brief    Blink LEDs using counting delays and CMSIS
 * @version  V1.0
 * @date     31/10/2016
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

//@{
/// LED on GPIO Port E
#define LED_GREEN  (8)
/// LED on GPIO Port B
#define LED_RED    (2)
//@}

int main(void) {

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;   // Enable GPIO Port E
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIO Port B

    __DSB();

    /* Clear field and set desired value */
    GPIOB->MODER   = ((GPIOB->MODER&~(3<<(LED_RED*2)))    | (1<<(LED_RED*2)));   // Set to output
    GPIOB->OSPEEDR = ((GPIOB->OSPEEDR&~(3<<(LED_RED*2)))  | (3<<(LED_RED*2)));   // Set to high speed
    GPIOB->PUPDR   = ((GPIOB->PUPDR&~(3<<(LED_RED*2)))    | (1<<(LED_RED*2)));   // Set to pull up
    GPIOB->ODR    |= (1<<LED_RED);

    GPIOE->MODER   = ((GPIOE->MODER&~(3<<(LED_GREEN*2)))  | (1<<(LED_GREEN*2))); // Set to output
    GPIOE->OSPEEDR = ((GPIOE->OSPEEDR&~(3<<(LED_GREEN*2)))| (3<<(LED_GREEN*2))); // Set to high speed
    GPIOE->PUPDR   = ((GPIOE->PUPDR&~(3<<(LED_GREEN*2)))  | (1<<(LED_GREEN*2))); // Set to pull up
    GPIOE->ODR    &= ~(1<<LED_GREEN);



    for (;;) {
       ms_delay(500);
       GPIOB->ODR ^= (1 << LED_RED);       // Use XOR to toggle output
       GPIOE->ODR ^= (1 << LED_GREEN);     // Use XOR to toggle output
    }

    /* NEVER */
}
