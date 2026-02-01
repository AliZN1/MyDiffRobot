#ifndef CONFIG_HPP
#define CONFIG_HPP

/* -------- Constant Vales -------- */ 
#define DAC_resolution       ((1 << 12) - 1)
#define MAX_MSG_OUT_LEN      32 // serial_out maximum message length
#define MAX_MSG_IN_LEN       20 // serial read maximum message length
#define SERIAL_OUT_QUEUE_LEN 5
#define wheel_radius_m       0.03375
#define track_width_m        0.12342

const float pi = 3.14159265358979323846f;


/* -------- I/O pins -------- */ 
#define p_encoder_R     26
#define p_encoder_L     25
#define p_battery_level 34
#define p_ML_A          14 // left wheel
#define p_ML_B          32 // left wheel
#define p_MR_A          15 // right wheel
#define p_MR_B          33 // right wheel

/* -------- Delays -------- */ 
#define encodersManager_d  10
#define motorsController_d 10
#define serialReceiver_d   100
#define serialReadByte_d   1
#define serialPublisher_d  50
#define powerManager_d     2000

/* -------- Flags -------- */ 
extern bool encoder_publisher;
extern bool imu_publisher;
extern bool power_publisher;
extern bool controller_running;

/* -------- Structures -------- */ 
struct EncoderData_t {
    float right;
    float left;
};

struct Message_t {
      char text[MAX_MSG_OUT_LEN];
};

struct MotionType_t {
    uint8_t type;
    float value;
};

/* -------- Enumerates -------- */ 
enum SensorDataType { 
    Encoders_t, 
    IMU_t,
    Battery_t,
};

enum MotionType {
    motion_none,
    linear_dist,
    linear_speed,
    angular_deg,
};

enum serial_cmd {
    move_dis,
    move_speed,
    rotate_deg,
    encoder_pub,
    imu_pub,
};


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