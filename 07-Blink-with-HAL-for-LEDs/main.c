/**
 * @file     main.c
 * @brief    Use joystick buttons to control blinking
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     An abstraction layer for LEDs is used (led.[ch])
 *
 ******************************************************************************/

/** @mainpage 09-blink-enhanced: Using joystick and LEDS
 *   @par Description: Uses joystick buttons to control LEDs.
 *   @par An Hardware Abstraction Layer for LEDs is used.
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"

/**
 * @brief Control blinking period
 */

//@{
uint32_t semiperiod_max   = 5000;
uint32_t semiperiod_min   = 100;
uint32_t semiperiod_step  = 100;

uint32_t semiperiod_red   = 500;        // 1 sec
uint32_t semiperiod_green = 750;        // 1.5 sec
static uint32_t cnt_red = 0;
static uint32_t cnt_green = 0;
//@}

/// Blinking state (ative|inactive)
uint32_t blinking = 1;                  // blinking active

/// Debounce counter (only for CENTER, no neeed for others)
uint32_t debounce_counter = 0;

/**
 * @brief SysTick Handler
 *
 * @note Almost everything happens here
 */

void SysTick_Handler(void) {

    if( debounce_counter ) debounce_counter--;

    if( blinking ) {
        if( cnt_red == 0 ) {
            LED_Toggle(LED_RED);
            cnt_red = semiperiod_red;
        }
        cnt_red--;

        if( cnt_green == 0 ) {
            LED_Toggle(LED_GREEN);
            cnt_green = semiperiod_green;
        }
        cnt_green--;
    }
}


int main(void) {
uint32_t t;

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; /* Enable SysConfig, Comp, etc. */

    LED_Init(LED_ALL);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    for (;;) {}
}
