
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




#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "bitmanip.h"
#include "gpio.h"


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


int main(void) {

    //    SystemCoreClockSet(MSI48M_CLOCKSRC, 0, 3, 0);

    GPIO_Init(0,LED_GREEN|LED_RED);

    GPIO_Write(LED_GREEN,LED_RED);

    for (;;) {
       ms_delay(500);
       GPIO_Write(LED_RED,LED_GREEN);
       ms_delay(500);
       GPIO_Write(LED_GREEN,LED_RED);
    }
}
