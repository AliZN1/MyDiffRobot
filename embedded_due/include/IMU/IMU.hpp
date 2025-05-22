#ifndef IMU_HPP
#define IMU_HPP

#include <Wire.h>
#include "SerialComManager/SerialComManager.hpp"
#include "TaskManager/Task.hpp"

#define mpu_I2CAddr 0x68
#define mpu_RESET 0x00
#define mpu_CONFIG 0x1A
#define mpu_WHO_AM_I 0x75
#define mpu_PWR_MGMT_1 0x6B
#define mpu_ACCEL_XOUT_H 0x3B
#define mpu_GYRO_ZOUT_H 0x47
#define accel_LSB 16383.0
#define gyro_LSB 131.0
#define num_samples 100
#define max_loops_with_no_pub 2
#define bias_tolerance 20

class IMU: public Task{
private:
    SerialManager &serialManager;
    uint32_t last_time, current_time;
    uint8_t no_pub;
    float AccelX, AccelY, AccelZ;
    int16_t accelBias[3];
    uint16_t gyroBiasMax;
    uint16_t gyroBiasMin;
    uint16_t gyroBiasMean;
    float gyroYaw;
    bool isConnected;
    bool readAccel(int16_t (&arr)[3]);
    bool readGyro(int16_t &arr);
protected:
    void execute() override;
public:
    IMU(SerialManager &sm, uint16_t num_ticks);
    ~IMU();
    bool begin();
    bool resetPM();
    void calibrate();
    bool computeAccel();
    bool computeYaw();
    void pubAccel();
};

#endif