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

/** @mainpage 11-timetriggered: Blink LEDs
 *   @par Description: Uses joystick buttons to control LEDs.
 *        A Time Triggered approach is used.
 *        See Pont "Patterns for Time Triggered Embedded Systems"
 */

#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"
#include "joystick.h"
#include "tte.h"


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

/// Debounce counter (only for CENTER, no neeed for others)
uint32_t debounce_counter = 0;

/// Tick Counter: increments every ms
uint32_t msTick = 0;

/// Task number. Used to modify period
uint32_t taskno_red,taskno_green,taskno_button;

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
void Blink_Red(void) {
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
}

/**
 * @brief Blink Gree Task
 *
 * @note Called every semiperiod
 */
void Blink_Green(void) {
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
}

/**
 * @brief Button Processing Task
 *
 * @note polls the Center Button, debounce it and set blinking variable
 */
void ButtonProcessing(void) {
typedef enum { Q0, Q1, Q2 } state_t;
static state_t state = Q0;
static uint32_t debouncing_counter;
uint32_t b;

    switch(state) {
    case Q0: // Waiting Press of Button
        b = JoyStick_Read();
        if( b&JOY_CENTER ) {
            blinking = ! blinking;
            state = Q1;
            debouncing_counter = 10;
        }
        break;
    case Q1: // Ignore transitions
        if( --debouncing_counter ==  0 ) {
            state = Q2;
        }
        break;
    case Q2: // Wait release
        b = JoyStick_Read();
        if( (b&JOY_CENTER)==0 ) {
            state = Q0;
        }
        break;
    }
}


int main(void) {


    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    LED_Init(LED_ALL);

    JoyStick_Init();

    Task_Init();

    taskno_green = Task_Add(Blink_Green,semiperiod_green,0);
    taskno_red   = Task_Add(Blink_Red,semiperiod_red,0);

    taskno_button  = Task_Add(ButtonProcessing,10,5);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    for (;;) {
        Task_Dispatch();
    }
}
