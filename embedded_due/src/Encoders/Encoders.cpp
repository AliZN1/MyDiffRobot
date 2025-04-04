#include "Encoders/Encoders.hpp"

// ------------------------------------ Encoder
Encoder::Encoder(uint8_t pin_num): 
    pin(pin_num)
{
    last_angle = readAngle();
};

Encoder::~Encoder(){};

double Encoder::readAngle(){
    double read = analogRead(pin);
    if(read > uperLimit)
        read = uperLimit;
    else if(read < lowerLimit)
        read = lowerLimit;

    return (read - lowerLimit)/(uperLimit - lowerLimit) * 2 * pi;
}

void Encoder::setLimits(uint16_t min, uint16_t max){
    lowerLimit = min;
    uperLimit = max;
}

double Encoder::updateAngDisp(){
    double currentAngle = readAngle();
    double delta =  currentAngle - last_angle;
    last_angle = currentAngle;

    if(delta > pi){
        numTurns--;
    }
    else if (delta < -pi){
        numTurns++;
    }
    
    return numTurns * 2*pi + last_angle;
}

// ------------------------------------ EncoderManager
EncodersManager::EncodersManager(uint8_t pin_right, uint8_t pin_left):
    enc_R(pin_right),
    enc_L(pin_left)
{}

EncodersManager::~EncodersManager(){}

void EncodersManager::angularPos(){
    double angPos_R = enc_R.updateAngDisp();
    double angPos_L = enc_L.updateAngDisp();
    
    char result_str[100];
    sprintf(result_str, "%d", 10);
    Serial.print(result_str);
}