#include "Wheels/Encoders.hpp"

// ------------------------------------ Encoder
Encoder::Encoder(uint8_t pin_num, bool reverseDir): 
    pin(pin_num), uperLimit(1023), lowerLimit(0), omega(0.0), last_angle(0.0), numTurns(0)
{
    dir = reverseDir? -1 : 1;
};

Encoder::~Encoder(){};

void Encoder::setLimits(uint16_t min, uint16_t max){
    lowerLimit = min;
    uperLimit = max;
}

void Encoder::initLastAngle(){
    initAngle = readAngle();
    last_angle = initAngle;
    last_angPos = 0;
}

double Encoder::readAngle(){
    double read = analogRead(pin);
    if(read > uperLimit)
        read = uperLimit;
    else if(read < lowerLimit)
        read = lowerLimit;
    
    return (read - lowerLimit)/(uperLimit - lowerLimit) * 2 * pi;
}

double Encoder::updateAngDisp(){
    double currentAngle = readAngle();
    double delta =  currentAngle - last_angle;
    last_angle = currentAngle;
    // update angular displacement
    if(delta > pi){
        numTurns--;
    }
    else if (delta < -pi){
        numTurns++;
    }
    return (numTurns * 2*pi + currentAngle - initAngle) * dir;
    //compute the angular velocity of the wheel as well    
}

void Encoder::computeOmega(uint32_t &dt, double &cur_angPos){
    if(dt == 0)
        omega = 0;
    else
        omega = (cur_angPos - last_angPos) / static_cast<double>(dt) * 1e6;

    last_angPos = cur_angPos;
}

float Encoder::getOmega(){
    return omega;
}

// ------------------------------------ EncoderManager
EncodersManager::EncodersManager(uint8_t pin_right, uint8_t pin_left, SerialManager &sm, uint16_t num_ticks)
    : Task(num_ticks), enc_R(pin_right), enc_L(pin_left, true), serialManager(sm) {}

EncodersManager::~EncodersManager(){}

void EncodersManager::angularPos(double (&angPosArr)[2]){
    angPosArr[0] = enc_R.updateAngDisp();
    angPosArr[1] = enc_L.updateAngDisp();

    
    char result_str[maxNumChar];
    sprintf(result_str, "@E:%.4f,%.4f;\n", angPosArr[0], angPosArr[1]);
    serialManager.push_msg(result_str);
}

void EncodersManager::initLastAngles(){
    enc_R.initLastAngle();
    enc_L.initLastAngle();

    last_omegaStamp = micros();
}

void EncodersManager::execute(){
    double cur_angPosArr[2]; // current ang position for wheels right an left respectively
    angularPos(cur_angPosArr);

    uint32_t now = micros();
    uint32_t dt = now - last_omegaStamp;

    if(dt > omega_min_dt){
        enc_R.computeOmega(dt, cur_angPosArr[0]);
        enc_L.computeOmega(dt, cur_angPosArr[1]);
        last_omegaStamp = now;
    }
}

void EncodersManager::getAngVel(float (&angVel)[2]){
    angVel[0] = enc_R.getOmega();
    angVel[1] = enc_L.getOmega();
}
