#include "PowerManger/PowerManager.hpp"


/**
 * @short Creates an instance of the PowerManger class.
 * 
 * This class allow interacting with battery level and takes
 * necessary actions to protect the board from any power shortage.
 * Battery level is initialized in construction.
 * 
 * @param [in] serial_publisher A reference to an object of SerialPublisher 
 *      class to communicated with the main board.
 * @param [in] signal_pin Number of the pin connected to the battery 
 *      voltage signal.
 * @param [in] num_ticks The interval of executing "execute" function in millisecond.
 */
PowerManager::PowerManager(QueueHandle_t &msg_out_q)
    : serial_out_q(msg_out_q) { 
    readSignal();
}

PowerManager::~PowerManager(){

}

/**
 * This is an override function part of Task class which will be
 * executed periodically.
 */
void PowerManager::runTask(){
    while(1){
        readSignal();

        if(power_publisher)
            publish();

        vTaskDelay(pdMS_TO_TICKS(powerManager_d));
    }
}

/**
 * @short Reads the converted output voltage of the battery.
 * 
 * The function updates the battery voltage and check if it's lower
 * than the defined limit. If so, it publish a zero-volt signal.
 * The battery signal must be connected to an ADC pin of the microcontroller.
 */
void PowerManager::readSignal(){
    batteryV = analogRead(p_battery_level) * S2V_ratio;
    if(batteryV < lowerLimit) batteryV = 0;
}

/**
 * @short Publishes the latest battery voltage from 0 to 7.4 volt
 */
void PowerManager::publish(){
    Message_t msg;
    sprintf(msg.text, "@%02d:%.2f;\n", Battery_t, batteryV);
    xQueueSendToBack(serial_out_q, &msg, pdMS_TO_TICKS(0));
}

/**
 * @short
 */
void PowerManager::setLowerLimit(float min){
    if(min < LLV){
        lowerLimit = 5.5;
        return;
    }

    lowerLimit = min;
}