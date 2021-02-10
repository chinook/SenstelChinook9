
#ifndef MAIN_HPP
#define MAIN_HPP

#include <mbed.h>

#define TIME_ACQ 100
#define TIME_DATA_OUT 250
#define TIME_DATA_OUT_PITCH 200
#define TIME_LORA_DATA_OUT 250

//#define LED_DEBUG
#define RPM_KHZ_OUTPUT
#define WHEEL_RPM_KHZ_OUTPUT

// Defines for the CAN IDs
#define GEAR_CAN_ID 1
#define PITCH_CAN_ID 2
#define MAST_DIR_CAN_ID 3
#define PITCH_ALGO_CAN_ID 4
//#define MAST_MODE_CAN_ID 4
//#define CALIB_DONE_CAN_ID 5
#define ROTOR_RPM_CAN_ID 6
#define WIND_SPEED_CAN_ID 7
#define CURRENT_CAN_ID 8
#define VOLTAGE_CAN_ID 9
#define WHEEL_RPM_CAN_ID 10
#define WIND_DIR_CAN_ID 11
//#define ACQ_STAT_CAN_ID 12
//#define PITCH_MODE_CAN_ID 13
#define LOADCELL_CAN_ID 14
#define TORQUE_CAN_ID 15

#define MAX_TURB_RPM_VALUE 1200

enum MODE
{
  MODE_RACING,
  MODE_DEBUG
};
extern MODE mode;

struct SensorData {
    float torque = 0;
    float loadcell = 0;
    float wind_direction = 0;
    float wind_speed = 0;
    float rpm_rotor = 0;
    float rpm_wheels = 0;
    int gear = 0;
    int pitch = 0;
    float angle_mat = 0;
};
struct LoraData {
  SensorData sensors;
  bool bROPS = false;
  float avg_wind_direction = 0.0f;
  char mast_mode = 0;
  char pitch_mode = 0;
  unsigned errors = 0;
  float target_pitch = 0.0f;
};

// Outputs
extern Serial pc;
extern CAN can;

extern bool pitch_done;

extern float global_nb_steps;

#endif // MAIN_HPP
