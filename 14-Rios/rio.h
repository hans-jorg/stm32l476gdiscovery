#ifndef TTE_H
#define TTE_H

/*
 * Based on rio-scheduler from Vahid, Givargis, Miller
 * See http://www.riosscheduler.org/
 */
typedef uint32_t State_t;

typedef struct task {
   uint32_t period;                 // Rate at which the task should tick
   uint32_t elapsed;                // Time since task's last tick
   State_t state;                   // Current state
   State_t (*func)(State_t);        // Function to call for task's tick
} Task_t;

#ifndef TASK_MAXCNT
#define TASK_MAXCNT 10
#endif

extern Task_t   Task_List[TASK_MAXCNT];
extern uint32_t Task_Error;
extern uint32_t Task_Flag;

void     Task_Init(void);
void     Task_Dispatch(void);
void     Task_Update(void);
uint32_t Task_Delete(uint32_t);
int32_t  Task_Add( State_t (*task)(State_t), uint32_t period, uint32_t delay );

void     __attribute__((weak)) Task_Sleep(void);

#endif
