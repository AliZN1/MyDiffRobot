#include "Encoder/EncodersManager.hpp"


EncodersManager::EncodersManager(uint8_t pin_R, uint8_t pin_L, QueueHandle_t &msg_out_q, QueueHandle_t &encoder_data_q)
    : enc_R(pin_R), enc_L(pin_L), serial_out_q(msg_out_q), encoders_q(encoder_data_q){}

EncodersManager::~EncodersManager(){}

/**
 * @brief Performs the primary operations related to the encoders.
 * 
 * Updates the angular position and velocity of the  encoders. 
 * Publishes the results if activated.
 */
void EncodersManager::runTask(){
    while(1){
        angularPos();
        if(encoder_publisher) 
            pub_encoders();
        // angularVel(cur_angPosArr);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_TICK));
    }
}

/**
 * @brief Computes both encoders current angular position.
 * 
 * @param[out] angPosArr A 2-element array to store the current 
 * angular positions of the encoders.
 */
void EncodersManager::angularPos(){
    EncoderData_t enc_data;

    enc_data.right = enc_R.updateAngDisp();
    enc_data.left = enc_L.updateAngDisp();

    if(xQueueSendToBack(encoders_q, &enc_data, pdMS_TO_TICKS(0)) != pdPASS)
        Serial.println("fail to write enc data!");
}

/**
 * @brief Computes both encoders' current angular velocity.
 * 
 * To avoid division by zero, it returns `0` for both encoders' 
 * angular velocity if time stamps have less than 1 millisecond
 * difference.
 * 
 * @param[out] cru_angPosArr A 2-element array to store the 
 *      current angular position of the encoders.
 */
void EncodersManager::angularVel(){
    EncoderData_t enc_data;
    xQueuePeek(encoders_q, &enc_data, pdMS_TO_TICKS(0));

    uint32_t now = millis();
    uint32_t dt = now - last_omegaStamp;

    if(dt < omega_min_dt){
        enc_data.right = 0;
        enc_data.left = 0;
        return;
    }

    enc_R.computeOmega(dt, enc_data.right);
    enc_L.computeOmega(dt, enc_data.left);
    last_omegaStamp = now;
}

/**
 * @brief Stores the inital angle of both encoders.
 * 
 * Time stamp for angular velocity is initialized as well.
 */
void EncodersManager::initLastAngles(){
    enc_R.initLastAngle();
    enc_L.initLastAngle();

    last_omegaStamp = micros();
}

/**
 * @brief To access encoders' angular velocity
 * 
 * @param[out] angVel A 2-element array to store the current 
 *      angular velocity of the encoders.
 */
void EncodersManager::getAngVel(float (&angVel)[2]){
    angVel[0] = enc_R.getOmega();
    angVel[1] = enc_L.getOmega();
}

/**
 * @brief Publishes encoders' absolute angular position.
 * 
 * Publishes the encoder positions if the publisher is enabled 
 * (disabled by default).
 * 
 * @param[in] cur_angPosArr A 2-element array to store the current 
 *      angular position of the encoders.
 */
void EncodersManager::pub_encoders(){
    EncoderData_t enc_data;
    xQueuePeek(encoders_q, &enc_data, pdMS_TO_TICKS(0));

    Message_t msg;
    sprintf(msg.text, "@%02d:%.4f,%.4f;\n", Encoders_t, enc_data.right, enc_data.left);
    xQueueSendToBack(serial_out_q, &msg, pdMS_TO_TICKS(0));
}