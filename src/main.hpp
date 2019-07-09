
#ifndef MAIN_HPP
#define MAIN_HPP

#include <mbed.h>

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
static Serial pc(USBTX, USBRX, 115200);
static CAN can(PB_8, PB_9); // RD = PB8, TD = PB9

#endif // MAIN_HPP
