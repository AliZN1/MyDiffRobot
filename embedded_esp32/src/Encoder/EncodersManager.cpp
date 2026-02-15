#include "Encoder/EncodersManager.hpp"


EncodersManager::EncodersManager(uint8_t pin_R, uint8_t pin_L, QueueHandle_t &msg_out_q, QueueHandle_t &encoder_data_q)
    : enc_R(pin_R), enc_L(pin_L, true), serial_out_q(msg_out_q), encoders_q(encoder_data_q){}

EncodersManager::~EncodersManager(){}

/**
 * @brief Performs the primary operations related to the encoders.
 * 
 * Updates the angular position and velocity of the  encoders. 
 * Publishes the results if activated.
 */
void EncodersManager::runTask(){
    enc_L.setLimits(0, 4095);
    enc_R.setLimits(0, 4095);

    while(1){
        angularPos();
        if(encoder_publisher) 
            pub_encoders();

        vTaskDelay(pdMS_TO_TICKS(encodersManager_d));
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

    // if(!controller_running) return;
    
    if(xQueueOverwrite(encoders_q, &enc_data) != pdPASS)
        return;
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