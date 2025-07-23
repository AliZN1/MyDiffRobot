#ifndef MAIN_H
#define MAIN_H

#include <cstdint>
#include <SerialComManager/SerialComManager.hpp>
#include <Wheels/Encoders.hpp>
#include <Wheels/WheelsCon.hpp>
#include <TaskManager/TaskManager.hpp>
#include <IMU/IMU.hpp>
#include <PowerManger/PowerManager.hpp>

/* -------- I/O pins -------- */ 
#define p_encoder_R 26
#define p_encoder_L 25
#define p_battery_level 34
#define p_ML_A 14 // left wheel
#define p_ML_B 32 // left wheel
#define p_MR_A 15 // right wheel
#define p_MR_B 33 // right wheel

/* -------- cycles -------- */ 
#define c_1ms 1
#define c_100ms 100
#define c_20ms 20
#define c_2s 2000

#endif


/*

                                +--------------------------------+
                                | USB              ESP32 DEVKIT  |
                                |                            ___ |
                                | RESET                     |    |
                                | 3.3V                      |    |
                                | N/C                       |___ | 
                    PDB GND --> | GND                            |
              Right Encoder --> | IO26 (A0)                 VBAT | 
               Left Encoder --> | IO25 (A1)                   EN | 
              Battery Lever --> | IO34 (A2)                 VBUS |  
                                | IO39 (A3)                 IO13 |
                                | IO36 (A4)                 IO12 |
                                | IO4  (A5)                 IO27 | 
                                | IO5  (SCK)                IO33 | <-- MR_B
                                | IO18 (MOSI)               IO15 | <-- MR_A
                                | IO19 (MISO)               IO32 | <-- ML_B
                                | IO16                      IO14 | <-- ML_A
                                | I017            (I2C,SCL) IO22 | <-- IMU SCL
                                | IO21            (I2C,SDA) IO23 | <-- IMU SDA
                                +--------------------------------+

*/