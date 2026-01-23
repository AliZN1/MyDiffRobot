#ifndef TASK_HPP
#define TASK_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

class Task{
protected:
    Task(){}
public:
    virtual ~Task() = default;
    virtual void runTask() = 0;
    static void task(void* pv){
        static_cast<Task*>(pv)->runTask();
        vTaskDelete(nullptr);
    }
};

#endif