#include "scheduler.h"

uint8_t task_num;
scheduler schedulers[] = {
    // {Motor_Test,1000,0},
    {uart_task,10,0},
   {gps_task,100,0}  // 添加GPS任务，每100ms执行一次
};

void scheduler_init(){
    task_num = sizeof(schedulers) / sizeof(scheduler);
}

void scheduler_run(){
    for(uint8_t i = 0;i < task_num;i++){
        uint32_t now_time = HAL_GetTick();
        if(now_time - schedulers[i].last_run >= schedulers[i].run_ms){
            schedulers[i].last_run = now_time;
            schedulers[i].task_func();
        }
    }
}

