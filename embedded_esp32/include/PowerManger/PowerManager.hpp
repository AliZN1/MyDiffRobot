#ifndef POWERMANGER_HPP
#define POWERMANGER_HPP


#include <Arduino.h>
#include "TaskManager/Task.hpp"
#include "SerialComManager/SerialComManager.hpp"


#define S2V_ratio 1 // converts obtained signal from battery to its voltage
#define LLV 5.5 // hard coded lower limit voltage


class PowerManager: public Task{
private:
    SerialPublisher &serialPublisher;
    uint8_t signalPin;
    float batteryV; // battery voltage
    float lowerLimit;
    void read_signal();
    void publish();
protected:
    void execute() override;
public:
    PowerManager(SerialPublisher &serial_publisher, uint8_t signal_pin, uint16_t num_ticks);
    ~PowerManager();
    void setLowerLimit(float min);
};


#endif