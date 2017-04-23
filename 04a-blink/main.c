
/**
 * @file     main.c
 * @brief    Blink LEDs using interrupts and CMSIS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     The blinking frequency of the red LED depends on core frequency
 * @note     Direct access to registers
 * @note     No library used
 *
 *
 ******************************************************************************/

/** @mainpage 04-blink: Using interrutps
 *   @par Description: Blinks LEDs using interrupts and CMSIS
 *   @note The LED blinks controlled by SysTickHandler
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "bitmanip.h"
#include "gpio.h"



/**
 * @brief LED Symbols
 *
 *     LEDs are in different ports.
 *     In order to avoid wrong use, the port identifier should be appended to the symbol.
 *
 *     It is necessary to have different symbols to configure the GPIO
 *     One to specify bit in the ODR register and another to specify a 2-bit wide field in
 *     MODER,OSPEER and PUPDR registers.
 *     To write on a 2-bit wide field it is necessary to erase all bits on it using a mask.
 *
 */
/* Bit numbers for LEDS
 * LED     GPIO      Pin
 * Green   GPIOE      8
 * Red     GPIOB      2
 */

#define LED_GREEN   GPIO_MKWORD(BIT(8),0)
#define LED_RED     GPIO_MKWORD(0,BIT(2))

//@}

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
        GPIO_Toggle(LED_GREEN);
        cntgreen = 999;
    } else {
        cntgreen--;
    }

    if( cntred == 0 ) {
        GPIO_Toggle(LED_RED);
        cntred = 1200;
    } else {
        cntred--;
    }
}


int main(void) {

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    SysTick_Config(SystemCoreClock/1000);

    GPIO_Init(0,LED_GREEN|LED_RED);

    GPIO_Write(LED_GREEN,LED_RED);

    for (;;) {}
}
