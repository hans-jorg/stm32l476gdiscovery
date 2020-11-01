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

#include "ucos_ii.h"
#include "led.h"
#include "joystick.h"


/**
 * @brief Semiperiod values for LED1 and LED2
 */
//{
#define DELAY_GREEN       750
#define DELAY_RED        1750
//}


/**
 * Stacks for tasks
 */
//{
#define TASKSTART_STACKSIZE 100
#define TASKGREENLED_STACKSIZE 100
#define TASKREDLED_STACKSIZE 100

static OS_STK TaskStartStack[TASKSTART_STACKSIZE];
static OS_STK TaskGreenLEDStack[TASKGREENLED_STACKSIZE];
static OS_STK TaskRedLEDStack[TASKREDLED_STACKSIZE];

//}


/**
 * @brief  Task for blinking the Green LED
 *
 * @note   LED_Init must be called before
 */

void TaskGreenLED(void *param) {

    while(1) {
        LED_Toggle(LED_GREEN);
        OSTimeDly(DELAY_GREEN);
    }
}

/**
 * @brief  Task for blinking LED2
 *
 * @note   LED_Init must be called before
 */

void TaskRedLED(void *param) {

    while(1) {
        LED_Toggle(LED_RED);
        OSTimeDly(DELAY_RED);
    }
}


#define DEBOUNCE_TIME 40

volatile int blinking = 0;

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

/**
 * @brief   Board Support Package (BSP)
 */
typedef unsigned int CPU_INT32U;

void  OS_CPU_TickInit (CPU_INT32U  tick_rate)
{
    CPU_INT32U  cnts;
    CPU_INT32U  cpu_freq;

    cpu_freq = SystemCoreClock;
    cnts     = (cpu_freq / tick_rate);                          /* Calculate the number of SysTick counts               */

    OS_CPU_SysTickInit(cnts);                                   /* Call the Generic OS Systick initialization           */
}

/**
 * @brief  Task for starting other tasks
 *
 * @note   It is recommended to create tasks when uc/os is already running.
 *         This enable the calibration of Stats module.
 */


void TaskStart(void *param) {

    /* Configure LEDs */
    LED_Init(LED_GREEN|LED_RED);
    LED_Write(0,LED_GREEN|LED_RED);                             // Turn them on

    // Set clock source to external crystal: 48 MHz
//    (void) SystemCoreClockSet(CLOCK_HFE,1,1);

//    SysTick_Config(SystemCoreClock/OS_TICKS_PER_SEC);         // Initialize the Tick interrupt (CMSIS way)

    OS_CPU_TickInit(OS_TICKS_PER_SEC);                          // Initialize the Tick interrupt (uCOS way)

#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                               // Determine CPU capacity
#endif

    // Create a task to blink Green LED
    OSTaskCreate(   TaskGreenLED,                               // Pointer to task
                    (void *) 0,                                 // Parameter
                    (void *) &TaskGreenLEDStack[TASKGREENLED_STACKSIZE-1],
                                                                // Initial value of SP
                    TASKGREENLED_PRIO);                         // Task Priority/ID

    // Create a task to blink Red LED
    OSTaskCreate(   TaskRedLED,                                 // Pointer to task
                    (void *) 0,                                 // Parameter
                    (void *) &TaskRedLEDStack[TASKREDLED_STACKSIZE-1],
                                                                // Initial value of SP
                    TASKREDLED_PRIO);                           // Task Priority/ID

    // Effectively starting uC/OS
    __enable_irq();

    OSTaskDel(OS_PRIO_SELF);                                    // Kill itself. Task should never return
}

/**
 * @brief main routine
 */
int main(void) {

    // Initialize uc/os II
    OSInit();

    // Create a task to start the other tasks
    OSTaskCreate(   TaskStart,                                          // Pointer to function
                    (void *) 0,                                         // Parameter for task
                    (void *) &TaskStartStack[TASKSTART_STACKSIZE-1],    // Initial value of SP
                    TASKSTART_PRIO);                                    // Task Priority/ID

    // Enter uc/os and never returns
    OSStart();

}
