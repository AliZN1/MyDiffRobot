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
EncodersManager::EncodersManager(uint8_t pin_right, uint8_t pin_left, SerialManager &sm, uint16_t num_ticks)
    : Task(num_ticks), enc_R(pin_right), enc_L(pin_left), serialManager(sm) {}

EncodersManager::~EncodersManager(){}

void EncodersManager::angularPos(){
    double angPos_R = enc_R.updateAngDisp();
    double angPos_L = enc_L.updateAngDisp();
    
    char result_str[numChar];
    sprintf(result_str, "@E:%.4f,%.4f;\n", angPos_R, angPos_L);
    serialManager.push_msg(result_str);
}

void EncodersManager::execute(){
    angularPos();
}