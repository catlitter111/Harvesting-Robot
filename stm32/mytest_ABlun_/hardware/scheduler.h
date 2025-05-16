#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "bsp_system.h"

typedef struct scheduler{

    void (*task_func)(void);
    uint32_t run_ms;
    uint32_t last_run;
    
}scheduler;

void scheduler_init(void);
void scheduler_run(void);

#endif

