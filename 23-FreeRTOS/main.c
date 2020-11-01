/**
 * @file     main.c
 * @brief    Blinker with FreeRTOS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     FreeRTOS obtained in http:/www.freertos.org
 *
 ******************************************************************************/


/** @mainpage 16-freertos: Blink LEDs using FreeRTOS
 *   @par Description: Uses joystick buttons to control LEDs.
 *        A Hardware Abstraction Layer for LEDs and joystick is used.
 *   @par Uses FreeRTOS
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"
#include "joystick.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief Control blinking period
 */

//@{
uint32_t semiperiod_max   = 5000;
uint32_t semiperiod_min   = 100;
uint32_t semiperiod_step  = 100;

uint32_t semiperiod_red   = 250;
uint32_t semiperiod_green = 1000;

//static uint32_t cnt_red = 0;
//static uint32_t cnt_green = 0;
//@}

/// Blinking state (ative|inactive)
uint32_t blinking = 1;                  // blinking active

/// Tick Counter: increments every 1 ms
uint32_t msTick = 0;

/// Specifies which form of Delay is used
#define USE_TASKDELAYUNTIL 1

/**
 * @brief SST Tasks
 *
 */
//@{
void Task_Blink_Red(void *pvParameters){
const portTickType xFrequency = 1000;
portTickType xLastWakeTime=xTaskGetTickCount();

    while(1) {
        if( blinking ) {
            LED_Toggle(LED_RED);
#ifdef TASKDELAYUNTIL
            vTaskDelayUntil(&xLastWakeTime,xFrequency);
#else
            vTaskDelay(semiperiod_red);
#endif
        } else {
            LED_Write(0,LED_RED);
        }
    }
}

void Task_Blink_Green(void *pvParameters){
const portTickType xFrequency = 500;
portTickType xLastWakeTime=xTaskGetTickCount();

    while(1) {
        if( blinking ) {
            LED_Toggle(LED_GREEN);
#ifdef TASKDELAYUNTIL
            vTaskDelayUntil(&xLastWakeTime,xFrequency);
#else
            vTaskDelay(semiperiod_green);
#endif
        } else {
            LED_Write(0,LED_GREEN);
        }
    }
}


#define DEBOUNCE_TIME 40


void CenterButtonProc(void) {

        blinking = ! blinking;

}
//@?

/// Callback routines
static JoyStickCallBack joystick_callback = {
      .CenterButtonPressed = CenterButtonProc
//    .LeftButtonPressed   = LeftButtonProcessing,
//    .RightButtonPressed  = RightButtonProcessing,
//    .UpButtonPressed     = UpButtonProcessing,
//    .DownButtonPressed   = CenterButtonProc
};


int main(void) {


    SystemCoreClockSet(MSI48M_CLOCKSRC,0,2,0);

    APBPeripheralClockSet(0,0); /* Enable APBx */

    LED_Init(LED_ALL);

    JoyStick_Init(&joystick_callback);

 //   SysTick_Config(SystemCoreClock/1000);   /* 1 ms */

    xTaskCreate(Task_Blink_Red,"Red", 1000,0,1,0);
    xTaskCreate(Task_Blink_Green,"Green", 1000,0,2,0);

    vTaskStartScheduler();

    while(1) {}
}
