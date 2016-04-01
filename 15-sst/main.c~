/**
 * @file     main.c
 * @brief    Blinker with SST
 * @version  V1.0
 * @date     23/01/2016
 *
 * @note     SST for ARM obtained in https://github.com/upiitacode/SST_ARM
 *
 ******************************************************************************/


/** @mainpage 15-sst: Blink LEDs using SST
 *   @par Description: Uses joystick buttons to control LEDs.
 *        A Hardware Abstraction Layer for LEDs and joystick is used.
 *   @par Based on Super Simple Tasker (SST) of Ward/Samek
 */

#include "stm32l476xx.h"
#include "system_stm32l476.h"

#include "led.h"
#include "joystick.h"
#include "sst.h"
#include "sst_port.h"
#include "sst_stdsignal.h"
#include "sst_usersignal.h"

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

/// Tick Counter: increments every 1 ms
uint32_t msTick = 0;

/**
 * @brief Priorities for tasks
 */
//@{
#define BLINK_GREEN_PRIO    1
#define BLINK_RED_PRIO      2
#define BUTTON_PRIO         3
#define ISR_TICK_PRIO       200
//@}

/// Queue size
#define QUEUE_SIZE 3

/// Queue
//@{
static SSTEvent BlinkGreenQueue[QUEUE_SIZE];
static SSTEvent BlinkRedQueue[QUEUE_SIZE];
//@}

/// State definition
//@{
#define Q0     0
#define Q1     1
#define Q2     2
#define Q3     3
//@}
/**
 * @brief SysTick Handler
 *
 * @note Almost everything happens here
 */

void SysTick_Handler(void) {
int pin;

    SST_ISR_ENTRY(pin,ISR_TICK_PRIO);
//  SST_post(myTask_ID,1,0);
    msTick++;
    SST_post(BLINK_GREEN_PRIO, ISR_TICK_SIG, 0);
    SST_post(BLINK_RED_PRIO, ISR_TICK_SIG, 0);
//    SST_post(BUTTON_PRIO, ISR_TICK_SIG, 0);
    SST_ISR_EXIT(pin,(SCB->ICSR = SCB_ICSR_PENDSVSET_Msk));

}

/**
 * @brief SST Tasks
 *
 */
//@{
void Task_Blink_Red(SSTEvent event){
static uint32_t lasttick = 0;

    if(event.sig!=SST_SIGNAL_TASKINIT) {
        if( blinking ) {
            // static int state = 0
            // if (state == 0) {
            //    LED_Write(LED_RED,0);
            // } else {
            //    LED_Write(0,LED_RED);
            // }
            //
            if( msTick > (lasttick+semiperiod_red) ) {
                LED_Toggle(LED_RED);
                lasttick = msTick;
            };
        } else {
            LED_Write(0,LED_RED);
        }
    }
}

void Task_Blink_Green(SSTEvent event){
static uint32_t lasttick = 0;

    if(event.sig!=SST_SIGNAL_TASKINIT) {
        if( blinking ) {
            // static int state = 0
            // if (state == 0) {
            //    LED_Write(LED_GREEN,0);
            // } else {
            //    LED_Write(0,LED_GREEN);
            // }
            //
            if( msTick > (lasttick+semiperiod_green) ) {
                LED_Toggle(LED_GREEN);
                lasttick = msTick;
            };
        } else {
            LED_Write(0,LED_GREEN);
        }
    }

}


#define DEBOUNCE_TIME 40

void Task_Button(SSTEvent event) {

    if(event.sig!=SST_SIGNAL_TASKINIT){
/*
        PT_WAIT_UNTIL(pt,JoyStick_Read()&JOY_CENTER);
        blinking = ! blinking;
        tstart = msTick;
        PT_WAIT_UNTIL(pt,((msTick-tstart)>=DEBOUNCE_TIME));
        PT_WAIT_UNTIL(pt,!(JoyStick_Read()&JOY_CENTER));
*/
    }
}
//@}

/**
 * @brief Center button callback routine
 *
 * @note  Just toggle blinking on/off
 */
void CenterButtonProc(void) {

        blinking = ! blinking;

}

/// Callback table
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

    SST_init();

    //SST_task(myTask,myTask_ID, myTask_EQ, myTask_EVQL, SST_SIGNAL_TASKINIT, 0);

    SST_task(Task_Blink_Green, BLINK_GREEN_PRIO, BlinkGreenQueue, QUEUE_SIZE,
             SST_SIGNAL_TASKINIT, 0);
    SST_task(Task_Blink_Red, BLINK_RED_PRIO,     BlinkRedQueue,   QUEUE_SIZE,
             SST_SIGNAL_TASKINIT, 0);
    SST_task(Task_Button, BUTTON_PRIO,           0, 0, SST_SIGNAL_TASKINIT, 0);

    SysTick_Config(SystemCoreClock/1000);   /* 1 ms */


    SST_run();
}
