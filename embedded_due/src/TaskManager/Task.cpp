#include "TaskManager/Task.hpp"

Task::Task(uint16_t numTicks): period(numTicks), lastRun(0) {}

Task::~Task(){}

void Task::run(uint32_t currentTime){
    if(currentTime - lastRun >= period * baseTick){
        lastRun = currentTime;
        execute();
    }
}