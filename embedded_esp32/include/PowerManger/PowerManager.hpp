#ifndef POWERMANGER_HPP
#define POWERMANGER_HPP


#include <Arduino.h>
#include "Task.hpp"
#include "freertos/queue.h"
#include "Config.hpp"


#define S2V_ratio 1 // converts obtained signal from battery to its voltage
#define LLV 5.5 // hard coded lower limit voltage


class PowerManager: public Task{
private:
    float batteryV; // battery voltage
    float lowerLimit;
    QueueHandle_t serial_out_q;
    void runTask() override;
    void readSignal();
    void publish();
public:
    PowerManager(QueueHandle_t &msg_out_q);
    ~PowerManager();
    void setLowerLimit(float min);
};


#endif