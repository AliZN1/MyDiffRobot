#ifndef TASK_HPP
#define TASK_HPP

#include <cstdint>
#include <Config.hpp>


class Task{
private:
    uint16_t period;
    uint32_t lastRun;
protected:
 virtual void execute() = 0;
public:
    Task(uint16_t numTicks);
    ~Task();
    virtual void run(uint32_t currentTime);
};

#endif