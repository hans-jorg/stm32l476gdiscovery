/**
 * @file     main.c
 * @brief    Blink leds using a cooperative Run To Completion (RTC) Kernel
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     Direct access to registers
 * @note     No library except CMSIS is used
 * @note     An abstraction layer for LEDs and joystick is used (led.[ch])
 *
 ******************************************************************************/

/** @mainpage 13-rios: Blink LEDs
 *   @par Description: Uses joystick buttons to control LEDs.
 *        - A Time Triggered approach is used.
 *        - See www.riosscheduler.org
 *        - See RIOS: A Lightweight Task Scheduler for Embedded Systems.
 */
#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"
#include "joystick.h"
#include "rio.h"


/**
 * @brief Control blinking period
 */

//@{
uint32_t semiperiod_max   = 5000;
uint32_t semiperiod_min   = 100;
uint32_t semiperiod_step  = 100;

uint32_t semiperiod_red   = 500;        // 1 sec
uint32_t semiperiod_green = 750;        // 1.5 sec

//static uint32_t cnt_red = 0;
//static uint32_t cnt_green = 0;
//@}

/// Blinking state (ative|inactive)
uint32_t blinking = 1;                  // blinking active

/// Debounce counter
uint32_t debounce_counter = 0;

/// Task number
uint32_t taskno_green, taskno_red, taskno_button;

/// Tick Counter: increments every 1 ms
uint32_t msTick = 0;

/**
 * @brief SysTick Handler
 *
 * @note Almost nothing happens here
 */
void SysTick_Handler(void) {

    msTick++;
    Task_Update();

}

/**
 * @brief Blink Red Task
 *
 * @note Called every semiperiod
 */
State_t
Blink_Red(State_t state) {
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
    return 0;
}

/**
 * @brief Blink Green Task
 *
 * @note Called every semiperiod
 */
State_t
Blink_Green(State_t state) {
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
    return 0;
}

/**
 * @brief Button Task
 *
 */
///@{
#define Q0    0
#define Q1    1
#define Q2    2

State_t
ButtonProcessing(State_t state) {
State_t nextstate;
static uint32_t debouncing_counter;
uint32_t b;

    switch(state) {
    case Q0: // Waiting Press of Button
        b = JoyStick_Read();
        if( b&JOY_CENTER ) {
            blinking = ! blinking;
            nextstate = Q1;
            debouncing_counter = 10;
        }
        break;
    case Q1: // Ignore transitions
        if( --debouncing_counter ==  0 ) {
            nextstate = Q2;
        }
        break;
    case Q2: // Wait release
        b = JoyStick_Read();
        if( (b&JOY_CENTER)==0 ) {
            nextstate = Q0;
        }
        break;
    }
    return nextstate;
}
///@}


int main(void) {


    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    LED_Init(LED_ALL);

    JoyStick_Init();

    Task_Init();

    taskno_green   = Task_Add(Blink_Green,semiperiod_green,0);
    taskno_red     = Task_Add(Blink_Red,semiperiod_red,0);

    taskno_button  = Task_Add(ButtonProcessing,10,5);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    for (;;) {
        Task_Dispatch();
    }
}
