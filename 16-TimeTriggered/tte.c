
/**
 *
 * @brief Time Triggered Executive
 *
 *
 * @note Based in Pattern on Time Triggered Embedded System of Michael Ponn
 *
 * @note In Engineering Reliable Embedded System there is a (better) version
 *
 */

#include <stdint.h>
#include "tte.h"

/// Task Info
typedef struct {
    void    (*task)(void);  // pointer to function
    // time unit is ticks
    uint32_t    period;     // period, i.e. time between activations
    uint32_t    delay;      // time to next activation
    uint32_t    runcnt;     // if greater than 1, overrun
} TaskInfo;

/// Default value for Task Info Table
#ifndef TASK_MAXCNT
#define TASK_MAXCNT 10
#endif

/// Task info table
static TaskInfo taskinfo[TASK_MAXCNT];

/**
 * @brief Task Init
 * @note  Initialization of TTE Kernel
 */
uint32_t
Task_Init(void) {
int i;
    for(i=0;i<TASK_MAXCNT;i++) {
        Task_Delete(i);
    }
    return 0;
}

/**
 * @brief Task Add
 *
 * @note Add task to Kernel
 * @note delay can be used to serialize task activation and avoid clustering
 *       in a certain time
 */
int32_t
Task_Add( void (*task)(void), uint32_t period, uint32_t delay ) {
int taskno = 0;

    while( (taskno<TASK_MAXCNT) && taskinfo[taskno].task ) taskno++;
    if( taskno == TASK_MAXCNT ) return -1;

    taskinfo[taskno].task   = task;
    taskinfo[taskno].period = period;
    taskinfo[taskno].delay  = delay;
    taskinfo[taskno].runcnt = 0;

    return taskno;
}

/**
 * @brief Task Delete
 *
 * @note Remove task from Kernel
 * @note Use with caution because it modifies the scheduling calculation
 */
uint32_t
Task_Delete(uint32_t taskno) {

    taskinfo[taskno].task   = 0;
    taskinfo[taskno].period = 0;
    taskinfo[taskno].delay  = 0;
    taskinfo[taskno].runcnt = 0;
    return 0;
}

/**
 * @brief Task Dispatch
 *
 * @note Run tasks if they are ready
 * @note Must be called in main loop
 */
uint32_t
Task_Dispatch(void) {
int i;
TaskInfo *p;

    for(i=0;i<TASK_MAXCNT;i++) {
        p = &taskinfo[i];
        if( p->task ) {
            if( p->runcnt ) {
                p->task();
                p->runcnt--;
                if( p->runcnt == 0 && p->period == 0 ) {
                    Task_Delete(i);
                }
            }
        }

    }

    return 0;
}

/**
 * @brief Task Update
 *
 * @note Must be called only in Timer Interrupt Routine
 *
 */

uint32_t
Task_Update(void) {
int i;
TaskInfo *p;

    for(i=0;i<TASK_MAXCNT;i++) {
        p = &taskinfo[i];
        if( p->task ) {
            if( p->delay == 0 ) {
                p->runcnt++;
                if( p->period )
                    p->delay = p->period;
            } else {
                p->delay--;
            }
        }
    }

    return 0;
}

/**
 * @brief Task Modify Period
 *
 * @note CAUTION!!! It can modify all timing calculation
 *
 */
uint32_t
Task_ModifyPeriod(uint32_t taskno, uint32_t newperiod) {
uint32_t t = taskinfo[taskno].period;
    taskinfo[taskno].period = newperiod;
    return t;
}
