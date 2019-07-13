
#include <mbed.h>

#ifndef PITCH_HPP
#define PITCH_HPP

#define WHEEL_DIAMETER 0.48

#define PITCH_ABSOLUTE_ZERO 717455
#define PITCH_DRAPEAU 507521//856848

#define PITCH_MANUAL 0
#define PITCH_AUTOMATIC 1

namespace pitch
{

// Communication variables with drive pitch
static bool pitch_done = true;
static bool ROPS = false;

// Pitch mode
static unsigned pitch_mode = -1;

// Helper function for converting pitch value to encoder angle value
float pitch_to_angle(float pitch_value);

// Function called to set pitch to given angle of the blades
// The function will convert the delta angle from blades to encoder angle
// and send the appropriate CAN command.
void SendPitchAngleCmd(float current_pitch, float target_pitch_blades);

// Function called to automatically set the blades angle using
// an algorithm
// ** Master commandsfor auto pitch **
float AutoPitchWheelRPM(float current_pitch, float wheel_rpm, float& vehicle_speed);

// Function called to step to -15 degrees asap
void SendROPSCmd(float current_pitch, bool ROPS);

// Function called to set mode to automatic or manual
void SetMode(unsigned mode);

}

#endif
