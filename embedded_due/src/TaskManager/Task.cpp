#include "TaskManager/Task.hpp"

Task::Task(uint16_t numTicks): period(numTicks), lastRun(0) {}

Task::~Task(){}

/**
 * @brief Executes task function periodically 
 *
 * Last time stamp and period are class members. This has to be a non-blocking function.
 * 
 * @param[in] currentTime current time in microSeconds.
 */
void Task::run(uint32_t currentTime){
    if(currentTime - lastRun >= period * baseTick){
        lastRun = currentTime;
        execute();
    }
}