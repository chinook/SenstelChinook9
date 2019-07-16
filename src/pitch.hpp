
#include <mbed.h>

#ifndef PITCH_HPP
#define PITCH_HPP

#define WHEEL_DIAMETER 0.48

#define PITCH_ABSOLUTE_ZERO 736948//717455
#define PITCH_DRAPEAU 540470//507521//856848

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
float AutoPitchWheelRPM(float current_pitch, float wheel_rpm, float wind_speed, float& vehicle_speed);

// Function called to step to -15 degrees asap
void SendROPSCmd(float current_pitch, bool ROPS);

// Function called to set mode to automatic or manual
void SetMode(unsigned mode);



// Valeurs de pitch selon algo de Olivier Miclette - 2019

static float valeurs[19][10] =
{
	{ 0.80132601 , -12.05194147 ,  73.51088121 , -233.12997924 ,  408.53646373 , -380.47151190 , 147.26981778, 1.0,  3.6,  7.3}, // 3ms
	{ 0.05111249 , -1.24500934  ,  11.45706927 , -52.99580978  ,  133.62850982 , -177.39407217 , 95.23513653,  1.1,  4.3,  7.3}, // 3.5ms
	{ 0.14465332 , -2.84529622  ,  22.76613828 , -95.11790258  ,  220.76435690 , -273.55802234 , 141.28374532, 1.3,  4.9,  7.1}, // 4ms
	{ 0.14114246 , -3.00887962  ,  25.98147226 , -116.37160891 ,  286.54971186 , -371.66325896 , 199.52306314, 1.4,  5.4,  7.35}, // 4.5ms
	{ 0.08100889 , -1.90957590  ,  18.16145684 , -89.19494622  ,  239.67627905 , -337.97187115 , 197.47928182, 1.5,  6.0,  7.14}, // 5ms
	{ 0.04630442 , -1.20119013  ,  12.57380455 , -67.97495431  ,  201.06748010 , -312.03508749 , 200.70969986, 1.6,  6.5,  7.36}, // 5.5ms
	{ 0          ,             0,  0.04425599  , -1.00730552   ,  8.62474140   , -32.22676153  , 42.28792424,  1.9,  7.1,  7.0}, //6ms
	{ 0.01009455 , -0.31147825  ,  3.89511506  , -25.33657222  ,  91.20444287  , -174.97344385 , 141.31311881, 2.0,  7.8,  7.14}, // 6.5ms
	{ 0.00853232 , -0.27691272  ,  3.63657294  , -24.78115533  ,  93.15203062  , -186.11655349 , 156.68751850, 2.1,  8.4,  7.28}, // 7ms
	{ 0.00213626 , -0.07624901  ,  1.10673884  , -8.44230389   ,  36.44320156  , -86.81456881  , 89.69360917,  2.3,  9.0,  6.93}, // 7.5ms
	{-0.00067102 ,  0.02349653  , -0.31698054  ,  1.94501047   , -4.06122061   , -7.91703893   , 31.36269996,  2.4,  9.8,  7.07}, // 8ms
	{ 0.00040711 , -0.01750386  ,  0.30774066  , -2.90052504   ,  16.00101297  , -50.17808637  , 68.00755451,  2.6,  10.1, 7.21}, // 8.5ms
	{ 0.00001960 ,  0.00003534  , -0.00904358  ,  0.01133204   ,  1.76675299   , -15.82448314  , 37.45023635,  2.8,  10.8, 7.35}, // 9ms
	{-0.00035672 ,  0.01529834  , -0.25962690  ,  2.14274997   , -8.10711589   ,  7.39386274   , 17.11329103,  2.8,  11.2, 7.5}, // 9.5ms
	{ 0.00059250 , -0.02913583  ,  0.58281314  , -6.08754578   ,  35.32229716  , -109.42320907 , 141.28374544, 3.0,  12.0, 7.07}, // 10ms
	{ 0.00044213 , -0.02282869  ,  0.47948181  , -5.25865092   ,  32.03836477  , -104.21258006 , 141.28374545, 3.15, 12.6, 7.07}, // 10.5ms
	{ 0.00033445 , -0.01809106  ,  0.39806922  , -4.57366324   ,  29.19198110  , -99.47564456  , 141.28374540, 3.3,  13.2, 7.07}, // 11ms
	{ 0.00025615 , -0.01448566  ,  0.33322531  , -4.00266017   ,  26.70873133  , -95.15061664  , 141.28374552, 3.45, 13.8, 7.07}, // 11.5ms
	{ 0.00019843 , -0.01170904  ,  0.28106343  , -3.52288527   ,  24.52937292  , -91.18600725  , 141.28374508, 3.6,  14.4, 7.07} // 12ms
};

} // namespace pitch

#endif
