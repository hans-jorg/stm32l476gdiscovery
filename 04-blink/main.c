
/**
 * @file     main.c
 * @brief    Blink LEDs using interrupts and CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library used
 *
 *
 ******************************************************************************/

/** @mainpage 04-blink: Using interrutps
 *   @par Description: Blinks LEDs using interrupt and CMSIS
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
 * @brief Quick and dirty delay routine
 *
 * It gives approximately 1ms delay at 4 MHz (MSI)
 *
 */

void ms_delay(volatile int ms) {
   while (ms-- > 0) {
      volatile int x=7000;
      while (x-- > 0)
         __NOP();
   }
}


/**
 * @brief SysTick Handler routine
 *
 * SysTick is configured to generate an interrupt every 1 ms.
 *
 * @note two different rate for blinking
 */

void SysTick_Handler(void) {
static uint32_t cntred = 0;
static uint32_t cntgreen = 0;

    if( cntgreen == 0 ) {
        GPIOE->ODR ^= LED_GREEN;    // Use XOR to toggle output
        cntgreen = 999;
    } else {
        cntgreen--;
    }

    if( cntred == 0 ) {
        GPIOB->ODR ^= LED_RED;      // Use XOR to toggle output
        cntred = 1200;
    } else {
        cntred--;
    }
}

int main(void) {

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    SysTick_Config(SystemCoreClock/1000);

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;   // Enable GPIO Port E
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIO Port B

    __DSB();

    /* Clear field and set desired value */
    SETBITFIELD(GPIOB->MODER,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));       // Set to output
    SETBITFIELD(GPIOB->OSPEEDR,LED_RED_M2,BITVALUE(3,LED_RED_PIN));       // Set to high speed
    SETBITFIELD(GPIOB->PUPDR,LED_RED_M2,BITVALUE(1,LED_RED_PIN*2));       // Set to pull up
    GPIOB->ODR |= LED_RED;

    SETBITFIELD(GPIOE->MODER,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));   // Set to output
    SETBITFIELD(GPIOE->OSPEEDR,LED_GREEN_M2,BITVALUE(3,LED_GREEN_PIN*2)); // Set to high speed
    SETBITFIELD(GPIOE->PUPDR,LED_GREEN_M2,BITVALUE(1,LED_GREEN_PIN*2));   // Set to pull up
    GPIOE->ODR  &= ~LED_GREEN;



    for (;;) { __WFI(); }

}
