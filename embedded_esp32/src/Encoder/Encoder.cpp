#include "Encoder/Encoder.hpp"


Encoder::Encoder(uint8_t pin_num, bool reverseDir): 
    pin(pin_num), uperLimit(DAC_resolution), lowerLimit(0), last_angle(0.0), continuousAngle(0.0f)
{
    dir = reverseDir? -1 : 1;
    last_angle = readAngle();
};

Encoder::~Encoder(){};

/**
 * @brief Sets a limit on the expected analog input range from the encoder.
 * 
 * This helps achieve better resolution and allows the encoder angle
 * to approach values close to 0 and 2π more accurately.
 * 
 * @param[in] min Maximum expected analog signal from encoder.
 * @param[in] max Minimum expected analog signal from encoder.
 */
void Encoder::setLimits(uint16_t min, uint16_t max){
    lowerLimit = min;
    uperLimit = max;
}

/**
 * @brief Reads the corresponding analog input and returns the angle.
 * 
 * The analog input is processed based on the upper and lower limits, 
 * then converted to radians.
 * 
 * @return Encoder's current position [0, 2π].
 */
float Encoder::readAngle(){
    float read = analogRead(pin);
    
    if(read > uperLimit)
        read = uperLimit;
    else if(read < lowerLimit)
        read = lowerLimit;
    
    return (read - lowerLimit)/(uperLimit - lowerLimit) * 2 * pi;
}

/**
 * @brief Computes encoder absolute angular position.
 * 
 * Computes the angular difference each time it's called. Based on the 
 * difference, it detects full rotations and keeps track of the 
 * encoder's absolute position.
 * 
 * @return Encoder's current angular position relative to its 
 *      initial position.
 */
float Encoder::updateAngDisp(){
    float currentAngle = readAngle();
    float delta =  currentAngle - last_angle;

    // update angular displacement
    if (delta > pi)
        delta -= 2.0f * pi;
    else if (delta < -pi)
        delta += 2.0f * pi;
    
    continuousAngle += delta;
    last_angle = currentAngle;

    return continuousAngle * dir;
}