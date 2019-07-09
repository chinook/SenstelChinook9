
#include "pitch.hpp"

#include "main.hpp"

namespace pitch
{

//
// Automatic pitch angle calculation algorithms
//

// Algorithm no.1 based only on wheel RPM (vehicle speed)
float calc_pitch_angle_wheelRPM(float vehicle_speed) // speed in m/s
{
  // TODO: Algo de Miclaye
  return -(vehicle_speed <= 2.25) ? 6.9 :
          ((vehicle_speed >= 9.0) ? 0 :
          ((0.00213626 * pow(vehicle_speed, 6.0)) -
          (0.07624901 * pow(vehicle_speed, 5.0)) +
          (1.10673884 * pow(vehicle_speed, 4.0)) -
          (8.44230389 * pow(vehicle_speed, 3.0)) +
          (36.4432156 * pow(vehicle_speed, 2.0)) -
          (86.81456881 * vehicle_speed) +
           89.69360917));
}

// Function to convert from pitch value to encoder angle value
float pitch_to_angle(float pitch_value)
{
  const float MAX_PITCH_VALUE = 4194303;
  float delta_pitch = pitch_value - PITCH_ABSOLUTE_ZERO;

  float encodeur_angle = (delta_pitch * 360.0f) / MAX_PITCH_VALUE;

  // Loop around logic

  return encodeur_angle;
}

void SendPitchAngleCmd(float current_pitch, float target_pitch_blades)
{
  // Calculate current pitch angle from ABSOLUTE ZERO
  float current_pitch_angle_pales = (3.0f / 2.0f) * pitch_to_angle(current_pitch);

  // Calculate the delta pitch angle for the stepper motor
  float delta_pitch_angle_pales = target_pitch_blades - current_pitch_angle_pales;
  float delta_pitch_angle_encodeur = (2.0f / 3.0f) * delta_pitch_angle_pales;

  // Convert the target angle to stepper steps
  const float angle_mov_per_step = 1.8f / 293.89f;
  float nb_steps = (float)((int)(delta_pitch_angle_encodeur / angle_mov_per_step));

  // Set a maximum to the number of steps so that we don't overshoot too much
  // Plus, its safer in case of angle error
  if(nb_steps > 300) nb_steps = 300;

  // TODO: Add checks and validation of steps

  // Send the command to the stepper drive
  // Only send cmd if stepper has finished last command
  // Stepper drive will notice us when done with the pitch_done CAN message.
  //if(pitch_done)
  //{
    can.write(CANMessage(0x36, (char*)(&nb_steps), 4));
    pitch_done = false;
    wait_us(100);
  //}
}

float AutoPitchWheelRPM(float current_pitch, float wheel_rpm)
{
  // Get target pitch angle from ABSOLUTE ZERO
  float diameter = 0.48; // 162.5 cm rayon
  // TODO : Add calculation from wheel_rpm
  //static float vehicle_speed = 2.5f;
  float vehicle_speed = wheel_rpm * diameter * M_PI / 60.0f;

  float target_angle_blades = calc_pitch_angle_wheelRPM(vehicle_speed);

  // Send command to stepper drive to step to given blade angle
  SendPitchAngleCmd(current_pitch, target_angle_blades);

  return target_angle_blades;
}

void SendROPSCmd(float current_pitch, bool ROPS)
{
  struct
  {
    int brops;
    float drapeau_steps;
  } ROPS_data;

  if(ROPS)
  {
    // Calculate the amount of stepping to do to reach ROPS angle
    float current_pitch_angle_pales = (3.0f / 2.0f) * pitch_to_angle(current_pitch);
    float drapeau_pitch_angle_pales = (3.0f / 2.0f) * pitch_to_angle((float)PITCH_DRAPEAU);
    float delta_pitch_angle_pales = drapeau_pitch_angle_pales - current_pitch_angle_pales;
    float delta_pitch_angle_encodeur = (2.0f / 3.0f) * delta_pitch_angle_pales;

    const float angle_mov_per_step = 1.8f / 293.89f;
    float nb_steps = (float)((int)(delta_pitch_angle_encodeur / angle_mov_per_step));
    if(nb_steps > 300) nb_steps = 300;
    ROPS_data.drapeau_steps = nb_steps;
    ROPS_data.brops = 1;

    can.write(CANMessage(0x37, (char*)(&ROPS_data), 8));
    wait_us(100);
  }
  else
  {
    ROPS_data.drapeau_steps = 0.0f;
    ROPS_data.brops = 0;
    can.write(CANMessage(0x37, (char*)(&ROPS_data), 8));
    wait_us(100);
  }
}

void SetMode(unsigned mode)
{
  can.write(CANMessage(0x33, (char*)(&mode), 4));
  wait_us(100);
}

} // namespace pitch
