
/*
 *
 * Time Triggered Executive
 *
 *
 * Based in Pattern on Time Triggered Embedded System of Michael Ponn
 *
 * In Engineering Reliable Embedded System there is a (better) version
 *
 */

#include <stdint.h>

#include "rio.h"

//#include <stm32l476xx.h>
// From CMSIS Core_cmFunc.h
__attribute__((always_inline))
static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}
__attribute__((always_inline))
static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}


uint32_t Task_Error;
uint32_t Task_Flag;

Task_t Task_List[TASK_MAXCNT];

static uint32_t task_tickcounter = 0;



void
Task_Init(void) {
int i;
    for(i=0;i<TASK_MAXCNT;i++) {
        Task_Delete(i);
    }
    Task_Error = 0;
    Task_Flag  = 0;
;
}

int32_t
Task_Add( State_t (*task)(State_t), uint32_t period, uint32_t delay ) {
int taskno = 0;

    while( (taskno<TASK_MAXCNT) && Task_List[taskno].func ) taskno++;
    if( taskno == TASK_MAXCNT ) return -1;

    Task_List[taskno].func     = task;
    Task_List[taskno].period   = period;
    Task_List[taskno].elapsed  = delay;
    Task_List[taskno].state    = 0;

    return taskno;
}

/*
 * Not recommended !!!!!
 */
uint32_t
Task_Delete(uint32_t taskno) {

    Task_List[taskno].func   = 0;
    Task_List[taskno].period = 0;
    return 0;
}

void
Task_Dispatch(void) {
int i;
Task_t *p;

    for(i=0;i<TASK_MAXCNT;i++) {
        p = &Task_List[i];
        if( p->func ) {
            if( p->elapsed >= p->period ) {
                p->state = p->func(p->state);      // call task function
                p->elapsed = 0;
            }
            p->elapsed++;
        }
    }

    Task_Flag = 0;
    while( ! Task_Flag ) {
        Task_Sleep();
    }

}

void __attribute__((weak))
Task_Sleep(void)  {
    // Sleep
}

/*
 * Task Update
 *
 * Must be called only in Timer Interrupt Routine
 */

void
Task_Update(void) {

    task_tickcounter++;

    if (Task_Flag) {
        // Time Overrun
        Task_Error = 1;
    } else {
        Task_Flag = 1;
    }
    return;
}


/*
 * Task Modify Period
 *
 *
 * CAUTION: It can modify all timing calculation
 */
uint32_t
Task_ModifyPeriod(uint32_t taskno, uint32_t newperiod) {
uint32_t t = Task_List[taskno].period;

    Task_List[taskno].period = newperiod;

    return t;
}
