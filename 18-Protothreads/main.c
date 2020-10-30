/**
 * @file     main.c
 * @brief    Blink leds using a cooperative Run To Completion (RTC) Kernel (protothreads)
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     An abstraction layer for LEDs and joystick is used (led.[ch])
 *
 ******************************************************************************/

/** @mainpage 13-protothread: Blink LEDs
 *   @par Description: Uses joystick buttons to control LEDs.
 *        A Protothreads approach is used.
 *        See Adam Dunkels, Oliver Schmidt, Thiemo Voigt, Muneeb Ali. Protothreads:
 *        simplifying event-driven programming of memory-constrained embedded systems;
 *        Proceedings of the 4th International Conference on Embedded Networked
 *        Sensor Systems. Pages 29-42. 2006.
 */
#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"
#include "joystick.h"
#include "pt.h"

/// Protothreads context
struct pt pt_BlinkGreen, pt_BlinkRed, pt_ButtonProc;

/**
 * @brief Control blinking period
 */

//@{
uint32_t semiperiod_max   = 5000;
uint32_t semiperiod_min   = 100;
uint32_t semiperiod_step  = 100;

uint32_t semiperiod_red   = 250;        // 1 sec
uint32_t semiperiod_green = 1000;        // 1.5 sec

//static uint32_t cnt_red = 0;
//static uint32_t cnt_green = 0;
//@}

/// Blinking state (ative|inactive)
uint32_t blinking = 1;                  // blinking active

/// Tick Counter: increments every ms
uint32_t msTick = 0;

/**
 * @brief SysTick Handler
 *
 * @note Almost nothing happens here
 */
void SysTick_Handler(void) {

    msTick++;

}

/**
 * @brief Blink Red Task
 *
 * @note Called every semiperiod
 */
static
PT_THREAD(Blink_Red(struct pt *pt)) {
static uint32_t tstart;

    PT_BEGIN(pt);
    while( 1 ) {
        if( blinking )
            // static int state = 0
            // if (state == 0) {
            //    LED_Write(LED_RED,0);
            // } else {
            //    LED_Write(0,LED_RED);
            // }
            //
            LED_Toggle(LED_RED);
        else
            LED_Write(0,LED_RED);
        tstart = msTick;
        PT_WAIT_UNTIL(pt,((msTick-tstart)>=semiperiod_red));
    }
    PT_END(pt);
}

/**
 * @brief Blink Green Task
 *
 * @note Called every semiperiod
 */
static
PT_THREAD(Blink_Green(struct pt *pt)) {
static uint32_t tstart;

    PT_BEGIN(pt);
    while(1) {
        if( blinking )
            // static int state = 0
            // if (state == 0) {
            //    LED_Write(LED_GREEN,0);
            // } else {
            //    LED_Write(0,LED_GREEN);
            // }
            //
            LED_Toggle(LED_GREEN);
        else
            LED_Write(0,LED_GREEN);
        tstart = msTick;
        PT_WAIT_UNTIL(pt,((msTick-tstart)>=semiperiod_green));
    }
    PT_END(pt);
}
/// Debounce Time
#define DEBOUNCE_TIME 40
/**
 * @brief Button Task
 *
 * @note Called every 1 ms
 */
static
PT_THREAD(ButtonProc(struct pt *pt)) {
static uint32_t tstart;
uint32_t b;

    PT_BEGIN(pt);
    while(1) {
        PT_WAIT_UNTIL(pt,JoyStick_Read()&JOY_CENTER);
        blinking = ! blinking;
        tstart = msTick;
        PT_WAIT_UNTIL(pt,((msTick-tstart)>=DEBOUNCE_TIME));
        PT_WAIT_UNTIL(pt,!(JoyStick_Read()&JOY_CENTER));
    }
    PT_END(pt);
}

int main(void) {


    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    LED_Init(LED_ALL);

    JoyStick_Init();

    PT_INIT(&pt_BlinkGreen);
    PT_INIT(&pt_BlinkRed);
    PT_INIT(&pt_ButtonProc);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    for (;;) {
        PT_SCHEDULE(Blink_Green(&pt_BlinkGreen));
        PT_SCHEDULE(Blink_Red(&pt_BlinkRed));
        PT_SCHEDULE(ButtonProc(&pt_ButtonProc));
    }
}
