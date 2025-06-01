#include "IMU/IMU.hpp"

IMU::IMU(SerialPublisher &sp, uint16_t num_ticks)
    : Task(num_ticks), serialPublisher(sp), last_time(0), last_pubTime(0), publish(false) {
        gyroYaw = 0.0;
    }

IMU::~IMU(){}

bool IMU::begin(){
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    
    Wire.beginTransmission(mpu_I2CAddr);
    byte res = Wire.endTransmission();

    if(res != 0){
        isConnected = false;
        return 0;
    }

    isConnected = true;

    resetPM(); // reset the power management of mpu6050
    // initialize the gyro bias max and min
    int16_t gyroData;
    readGyro(gyroData);
    gyroBiasMax = gyroData; 
    gyroBiasMin = gyroData;
    //carry out he calibration
    calibrate();
    return 1;
}

void IMU::execute(){
    if(!isConnected) return;

    int succeed = computeYaw();
    // error += computeAccel();
    if(succeed != 1) return; // ensure every task succeed
    
    pubAccel(); 
}

bool IMU::resetPM(){
    Wire.beginTransmission(mpu_I2CAddr);
    Wire.write(mpu_PWR_MGMT_1);
    Wire.write(mpu_RESET);
    byte res = Wire.endTransmission(true);
    
    Wire.beginTransmission(mpu_I2CAddr); // Command to activate the low pass filter with 10Hz cut off freq
    Wire.write(mpu_CONFIG);
    Wire.write(0x05);
    Wire.endTransmission(true);

    return res == 0;
}

void IMU::calibrate(){
    int32_t total_accel_data[] = {0, 0, 0};
    int32_t total_gyro_data = 0;
    for(int i = 0; i < num_samples; i++){
        int16_t accel_data[3];
        int16_t gyro_data;
        readAccel(accel_data);
        readGyro(gyro_data);
        for(int j = 0; j < 3; j++)
            total_accel_data[j] += accel_data[j];
        total_gyro_data += gyro_data;    
        //update gyro bias max and min based on the readings
        if(gyro_data > gyroBiasMax && gyro_data < gyroBiasMax + bias_tolerance) // bias_tolerance privent too large or small value become bias max and min
            gyroBiasMax = gyro_data;
        else if(gyro_data < gyroBiasMin && gyro_data > gyroBiasMin - bias_tolerance)
            gyroBiasMin = gyro_data;

        delay(50); // enough delay to ensure mpu6050 returns updated values
    }
    for(int i = 0; i < 3; i++)
        accelBias[i] = total_accel_data[i] / num_samples;
    gyroBiasMean = total_gyro_data / (num_samples);
    
    accelBias[2] += (int16_t)accel_LSB; // The Z axis has to be -1 not zero as it's perpendicular to the round 

    char msg [maxNumChar];
    sprintf(msg, "IMU calibration is done!\n");
    serialPublisher.push_msg(msg);
}

bool IMU::readAccel(int16_t (&arr)[3]){
    if(!isConnected) return 0;

    Wire.beginTransmission(mpu_I2CAddr);
    Wire.write(mpu_ACCEL_XOUT_H); // Set starting register (0x3B)
    Wire.endTransmission(false); // Send REPEATED START, not STOP
    Wire.requestFrom(mpu_I2CAddr,6,true); // Request 6 bytes (Accel X, Y, Z)

    if(Wire.available() < 6){
        char msg [maxNumChar];
        sprintf(msg, "No data available for Accel!\n");
        serialPublisher.push_msg(msg);
        return 0;
    }

    arr[0] = (Wire.read() << 8 | Wire.read());
    arr[1] = (Wire.read() << 8 | Wire.read());
    arr[2] = (Wire.read() << 8 | Wire.read());

    return 1;
}

bool IMU::computeAccel(){
    int16_t accel_data[3];
    bool res = readAccel(accel_data);

    if(!res) return 0;

    AccelX = (accel_data[0] - accelBias[0]) / accel_LSB;
    AccelY = (accel_data[1] - accelBias[1]) / accel_LSB;
    AccelZ = (accel_data[2] - accelBias[2]) / accel_LSB;

    return 1;
}

bool IMU::readGyro(int16_t &arr){
    if(!isConnected) return 0;

    Wire.beginTransmission(mpu_I2CAddr);
    Wire.write(mpu_GYRO_ZOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(mpu_I2CAddr, 6, true);
    current_time = micros();

    if(Wire.available() < 6){
        char msg [maxNumChar];
        sprintf(msg, "No data available for Gyro!\n");
        serialPublisher.push_msg(msg);
        return 0;
    }

    arr = (Wire.read() << 8 | Wire.read());
    return 1;
}

bool IMU::computeYaw(){
    int16_t gyro_data;
    bool res = readGyro(gyro_data);
    
    if(!res) return 0;

    if(gyro_data >= gyroBiasMin && gyro_data <= gyroBiasMax) gyro_data = 0;
    else gyro_data -= gyroBiasMean;

    float GyroZ = gyro_data / gyro_LSB;
    float elapsedTime = (current_time - last_time) / 1000000.0;
    last_time = current_time;
    gyroYaw += GyroZ * elapsedTime;
    
    return 1;
}

void IMU::pubAccel(){
    if(!publish) return;

    uint32_t now = millis();
    uint32_t dt = now - last_pubTime;
    if(dt < pub_delay) return;// reduce the data publication freq

    char msg [maxNumChar];
    sprintf(msg, "@I:%.4f\n", gyroYaw);
    serialPublisher.push_msg(msg);
    last_pubTime = now;
}

void IMU::set_publish(bool state){
    publish = state;
}