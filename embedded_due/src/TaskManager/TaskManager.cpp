#include "TaskManager/TaskManager.hpp"


TaskManager::TaskManager(Task** taskList, uint8_t size)
    : prt_taskList(taskList), listSize(size) {}

TaskManager::~TaskManager(){}

/**
 * @brief Goes through the tasks list and runs them one after each other.
 */
void TaskManager::run(){
    uint32_t now = micros();
    
    for(int i = 0; i < listSize; ++i){
        prt_taskList[i] -> run(now);
    }
}