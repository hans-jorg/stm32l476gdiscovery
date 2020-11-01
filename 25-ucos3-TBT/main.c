/**
 * @file     main.c
 * @brief    Blinker with FreeRTOS
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     FreeRTOS obtained in http:/www.freertos.org
 *
 ******************************************************************************/


/** @mainpage ucos3: Blink LEDs using ucos3
 *   @par Description: Uses joystick buttons to control LEDs.
 *        A Hardware Abstraction Layer for LEDs and joystick is used.
 *   @par Uses ucos3
 */


#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "app_cfg.h"
#include "os.h"


#include "led.h"
#include "joystick.h"

/**
 *  Stop function. The parameter can indicate where it was called
 */
///@{
static int sn;

void Stop(int n) {
    sn = n;
    while(1) {}
}
///@}

/**
 * @brief Semiperiod values for LED1 and LED2
 */
///@{
#define DELAY_GREEN       750
#define DELAY_RED        1750
///@}


/**
 * Parameters for Task TaskStart
 */
///@{
#define TASKSTART_STACKSIZE 100
static CPU_STK TaskStart_Stack[TASKSTART_STACKSIZE];
static OS_TCB TaskStart_TCB;
///@}


/**
 * Parameters for Task TaskGreeLED
 */
///@{
#define TASKGREENLED_STACKSIZE 100
static CPU_STK TaskGreenLED_Stack[TASKGREENLED_STACKSIZE];
static OS_TCB TaskGreenLED_TCB;
///@}

/**
 * Parameters for Task TaskGreeLED
 */
///@{
#define TASKREDLED_STACKSIZE 100
static CPU_STK TaskRedLED_Stack[TASKREDLED_STACKSIZE];
static OS_TCB TaskRedLED_TCB;
///@}


/**
 * @brief  Task for blinking the Green LED
 *
 * @note   LED_Init must be called before
 */

void TaskGreenLED(void *param) {
OS_ERR err;

    while(1) {
        LED_Toggle(LED_GREEN);
        OSTimeDly(DELAY_GREEN,OS_OPT_TIME_DLY,&err);
        if( err != OS_ERR_NONE )
            Stop(4);
    }
}

/**
 * @brief  Task for blinking LED2
 *
 * @note   LED_Init must be called before
 */

void TaskRedLED(void *param) {
OS_ERR err;

    while(1) {
        LED_Toggle(LED_RED);
        OSTimeDly(DELAY_GREEN,OS_OPT_TIME_DLY,&err);
        if( err != OS_ERR_NONE )
            Stop(5);
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

#if 0
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
#endif

/**
 * @brief  Task for starting other tasks
 *
 * @note   It is recommended to create tasks when uc/os is already running.
 *         This enable the calibration of Stats module.
 */


void TaskStart(void *param) {
OS_ERR err;

    param = param;                                      // To avoid warning

    CPU_Init();

    /* Configure LEDs */
    LED_Init(LED_GREEN|LED_RED);
    LED_Write(0,LED_GREEN|LED_RED);                             // Turn them on

    // Set clock source to external crystal: 48 MHz
    // (void) SystemCoreClockSet(CLOCK_HFE,1,1);


    OSTaskCreate(   (OS_TCB *)      &TaskGreenLED_TCB,
                    (CPU_CHAR *)    "Task Green LED",
                    (OS_TASK_PTR )  TaskGreenLED,
                    (void *)        0,
                    (OS_PRIO )      TASKGREENLED_PRIO,
                    (CPU_STK *)     &TaskGreenLED_Stack[0],
                    (CPU_STK_SIZE)  0,
                    (CPU_STK_SIZE)  TASKGREENLED_STACKSIZE,
                    (OS_MSG_QTY )   0,
                    (OS_TICK )      0,
                    (void *)        0,
                    (OS_OPT)        (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                    (OS_ERR *)      &err
                );

    OSTaskCreate(   (OS_TCB *)      &TaskRedLED_TCB,
                    (CPU_CHAR *)    "Task Red LED",
                    (OS_TASK_PTR )  TaskRedLED,
                    (void *)        0,
                    (OS_PRIO )      TASKREDLED_PRIO,
                    (CPU_STK *)     &TaskRedLED_Stack[0],
                    (CPU_STK_SIZE)  0,
                    (CPU_STK_SIZE)  TASKREDLED_STACKSIZE,
                    (OS_MSG_QTY )   0,
                    (OS_TICK )      0,
                    (void *)        0,
                    (OS_OPT)        (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                    (OS_ERR *)      &err
                );

    // Effectively starting uC/OS
    __enable_irq();

     while (1) {
        OSTimeDlyHMSM(  (CPU_INT16U)    0,
                        (CPU_INT16U)    0,
                        (CPU_INT16U)    0,
                        (CPU_INT32U)    100,
                        (OS_OPT )       OS_OPT_TIME_HMSM_STRICT,
                        (OS_ERR *)      &err
                    );
        if( err != OS_ERR_NONE )
            Stop(1);

        LED_Toggle(LED_GREEN|LED_RED);
    }}

/**
 * @brief main routine
 */
int main(void) {
OS_ERR err;

    // Initialize uC/OS III
    OSInit(&err);
    if( err != OS_ERR_NONE )
        Stop(1);

    /* Configure LEDs */
    LED_Init(LED_GREEN|LED_RED);

    // Create a task to start the other tasks
    OSTaskCreate(   (OS_TCB   *)        &TaskStart_TCB,             // TCB
                    (CPU_CHAR *)        "App Task Start",           // Name
                    (OS_TASK_PTR )      TaskStart,                  // Function
                    (void*)             0,                          // Parameter
                    (OS_PRIO)           TASKSTART_PRIO,             // Priority (in app_cfg.h)
                    (CPU_STK *)         &TaskStart_Stack[0],         // Stack
                    (CPU_STK_SIZE)      TASKSTART_STACKSIZE/10,     // Slack
                    (CPU_STK_SIZE)      TASKSTART_STACKSIZE,        // Size
                    (OS_MSG_QTY )       0,
                    (OS_TICK )          0,
                    (void *)            0,
                    (OS_OPT )           (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                    (OS_ERR *)          &err
                );
    if( err != OS_ERR_NONE )
        Stop(2);

    // Enter uc/os and never returns
    OSStart(&err);
    if( err != OS_ERR_NONE )
        Stop(3);

}
