#ifndef ENCODERS_MANAGER_HPP
#define ENCODERS_MANAGER_HPP

#include "Encoder/Encoder.hpp"
#include "Task.hpp"
#include "freertos/queue.h"

#define omega_min_dt 100 // ms

class EncodersManager: public Task{
private:
    Encoder enc_R;
    Encoder enc_L;
    uint32_t last_omegaStamp;
    QueueHandle_t encoders_q;
    QueueHandle_t serial_out_q;
    void runTask() override;

public:
    EncodersManager( uint8_t pin_R, uint8_t pin_L, QueueHandle_t &msg_out_q, QueueHandle_t &encoder_data_q);
    ~EncodersManager();
    void angularPos();
    void angularVel();
    void initLastAngles();
    void getAngVel(float (&angVel)[2]); // return right and left omega respectively
    void pub_encoders();
};

#endif