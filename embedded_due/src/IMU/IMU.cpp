#include "IMU/IMU.hpp"

IMU::IMU(SerialPublisher &sp, uint16_t num_ticks)
    : Task(num_ticks), serialPublisher(sp), last_time(0), last_pubTime(0), publish(false) {
        gyroYaw = 0.0;
    }

IMU::~IMU(){}

/**
 * @brief Begins I2C connection and IMU setup.
 *
 * Begins I2C and ensures connection to IMU. Resets power management on MPU6050 and calibrates it.
 * 
 * @return `1` if the setup was successful, `0` otherwise. 
 */
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

/**
 * @brief Virtual override function to be run by task manager.
 *
 * Makes sure IMU is connected and computes the Yaw.
 *
 */
void IMU::execute(){
    if(!isConnected) return;

    int succeed = computeYaw();
    // error += computeAccel();
    if(succeed != 1) return; // ensure every task succeed
    
    pubAccel(); 
}

/**
 * @brief Resets the IMU power management to default.
 *
 * Changes power management to default for full performance. Also, activate MPU6050 built-in 
 * low pass filter.
 * 
 * @return `1` if MPU6050 power management was reset correctly, `0` otherwise.
 */
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


/**
 * @brief Calibrates IMU to find bias on gyroscope and accelerometer.
 *
 Calculates the mean of the accelerometer data to estimate and remove its bias.
 * For the gyroscope, determines the bias range using the maximum and minimum of the
 * collected data, helping to prevent drift caused by bias shifting.
 *
 */
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

/**
 * @brief Reads Accelerometer data along X, Y, Z axes from IMU.
 *
 * Requests 6 bytes of data via I2C from the MPU6050 and updates the input array
 * with the raw accelerometer values.
 * 
 * @param[in,out] arr A reference to an array of three `int16_t` elements.
 *      It will be updated with the raw accelerometer data for X, Y, and Z axes. 
 * @return `1` if data was successfully received, `0` otherwise.
 */
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

/**
 * @brief Computes acceleration in 3 axes according the reading resolution.
 *
 * The computed values are stored in three class member variables that hold the processed
 * accelerometer data.
 * 
 * @return `1` if data was successfully received, `0` otherwise.
 */
bool IMU::computeAccel(){
    int16_t accel_data[3];
    bool res = readAccel(accel_data);

    if(!res) return 0;

    AccelX = (accel_data[0] - accelBias[0]) / accel_LSB;
    AccelY = (accel_data[1] - accelBias[1]) / accel_LSB;
    AccelZ = (accel_data[2] - accelBias[2]) / accel_LSB;

    return 1;
}

/**
 * @brief Reads gyroscope data along Z axis from IMU.
 *
 * Requests 6 bytes of data via I2C from the MPU6050 and updates the input array
 * with the raw gyroscope value.
 * 
 * @param[in,out] arr A reference to a variable of `int16_t`.
 *      It will be updated with the raw gyroscope data for Z axis (yaw).
 * @return `1` if data was successfully received, `0` otherwise.
 */
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

/**
 * @brief Computes heading angle, along Z axis, according to gyroscope reading.
 *
 * If the gyroscope reading is outside the defined bias range, it is integrated over time
 * to update the class member variable that stores the yaw value. The resulting yaw angle
 * is wrapped to stay within the range [-180, 180] degrees.
 * 
 * @return `1` if data was successfully received, `0` otherwise.
 */
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
    wrapAng180(gyroYaw);
    
    return 1;
}

/**
 * @brief Publish IMU data on serial port.
 *
 * Adds the yaw reading to the serial communication manager's queue
 * to be printed over the serial interface.
 * 
 */
void IMU::pubAccel(){
    if(!publish) return;

    uint32_t now = millis();
    uint32_t dt = now - last_pubTime;
    if(dt < pub_delay) return;// reduce the data publication freq

    char msg [maxNumChar];
    sprintf(msg, "@%02d:%.4f;\n", IMU_t, gyroYaw);
    serialPublisher.push_msg(msg);
    last_pubTime = now;
}

/**
 * @brief Changes publishing state.
 * 
 * @param[in] state A boolean flag. If '0', serial interface 
 *      stop publishing IMU data; If `1`, it continues publishing.
 */
void IMU::set_publish(bool state){
    publish = state;
}

/**
 * @brief Returns current yaw value based on gyroscope readings.
 * 
 * @return The current value of the yaw angle, stored as a `float'.
 */
float IMU::get_gyroYaw(){
    return gyroYaw;
}

/**
 * @brief Converts angular position from [0, 360] range to  [-180, 180] range..
 * 
 * @param[in,out] arr Angular position with 'float' type. It will be updated with
 *      the new range wrapped angle.
 */
void wrapAng180(float &ang){
    ang = fmod(ang + 180, 360);
    if(ang < 0)
        ang += 360;
    ang -= 180;
}