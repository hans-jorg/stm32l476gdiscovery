/**
 * @file     main.c
 * @brief    Use joystick buttons to control blinking
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     An abstraction layer for LEDs and joystick is used (led.[ch])
 *
 ******************************************************************************/

/** @mainpage 10-joystick: Using joystick and LEDS
 *   @par Description: Uses joystick buttons to control LEDs.
 *        A Hardware Abstraction Layer for LEDs and joystick is used.
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"
#include "joystick.h"

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


/**
 * @brief Button callback routines
 *
 */
//@{
void
CenterButtonProcessing(void) {

    if( debounce_counter == 0 ) {
        if( blinking ) {
            /* Turn off all LEDS */
            LED_Write(0,LED_ALL);
            blinking = 0;
        } else {
            blinking = 1;
        }
        debounce_counter = 40;
        cnt_red = 0;
        cnt_green = 0;
    }

}

void
LeftButtonProcessing(void) {

    /* Decrease period of green LED blinking  */
    if( semiperiod_green > (semiperiod_min+semiperiod_step) )
        semiperiod_green -= semiperiod_step;

}

void
RightButtonProcessing(void) {

    /* Increase period of green LED blinking  */
    if( semiperiod_green < (semiperiod_max-semiperiod_step) )
        semiperiod_green += semiperiod_step;

}

void
UpButtonProcessing(void) {

    /* Decrease period of red LED blinking  */
    if( semiperiod_red > (semiperiod_min+semiperiod_step) )
        semiperiod_red -= semiperiod_step;

}

void
DownButtonProcessing(void) {

    /* Increase period of red LED blinking  */
    if( semiperiod_red < (semiperiod_max-semiperiod_step) )
        semiperiod_red += semiperiod_step;

}
//@}

/**
 * @brief Call back routines table
 */

static JoyStickCallBack joystick_callback = {
    .CenterButtonPressed = CenterButtonProcessing,
    .LeftButtonPressed   = LeftButtonProcessing,
    .RightButtonPressed  = RightButtonProcessing,
    .UpButtonPressed     = UpButtonProcessing,
    .DownButtonPressed   = DownButtonProcessing
};


int main(void) {

    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    LED_Init(LED_ALL);

    JoyStick_Init(&joystick_callback);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    for (;;) {}
}
