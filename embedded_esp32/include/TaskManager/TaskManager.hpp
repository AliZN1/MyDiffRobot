#ifndef TASKMANAGER_HPP
#define TASKMANAGER_HPP

#include "TaskManager/Task.hpp"
#include <Arduino.h>


class TaskManager{
private:
    Task** prt_taskList;
    const uint8_t listSize;
public:
    TaskManager(Task** taskList, uint8_t size);
    ~TaskManager();
    void run();
};

#endif