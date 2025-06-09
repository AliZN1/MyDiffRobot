#ifndef SERIALCOM_MANAGER_H
#define SERIALCOM_MANAGER_H

#include <Arduino.h>
#include <cppQueue.h>
#include <cstring>
#include <Config.hpp>
#include "TaskManager/Task.hpp"
#include "Wheels/WheelsCon.hpp"
#include "IMU/IMU.hpp"

#define QUEUE_SIZE 5

class WheelsCon;
class EncodersManager;
class IMU;

enum serial_cmd {
    move_speed,
    rotate_speed,
    encoder_pub,
    imu_pub,
};

enum SensorDataType { 
    Encoders_t, 
    IMU_t
};

class SerialPublisher: public Task{
private:
    UARTClass &serial;
    cppQueue msg_output;
protected:
    void execute() override;
public:
    SerialPublisher(UARTClass &b_serial, uint16_t num_tick);
    ~SerialPublisher();
    void push_msg(const char message[maxNumChar]);
    void send_msg();
};

class SerialReceiver: public Task{
private:
    UARTClass &serial;
    WheelsCon &wheelsCon;
    EncodersManager &encodersManager;
    IMU &imu;
    char inputBuffer[maxNumChar];
    uint8_t inputByteIndex;
    void resetInputBuffer();
protected:
    void execute() override;
public:
    SerialReceiver(UARTClass &b_serial, uint16_t num_tick, WheelsCon &wheelsController, EncodersManager &encoders_manager, IMU &IMU_sensor);
    ~SerialReceiver();
    void read_msg();
    void processMessage(const char (&msg)[maxNumChar]);
};

#endif