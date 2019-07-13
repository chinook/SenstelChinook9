/*
 *   Chinook ÉTS (C) 2019
 *
 *   Marc Beaulieu
 *   Alfred Morel-Quintin
 */

#include "mbed.h"
//#include "BufferedSerial.h"

#include "lora.hpp"
#include "main.hpp"
#include "pitch.hpp"

#define LED_DEBUG

#define TIME_ACQ 50
#define TIME_DATA_OUT 500
#define TIME_DATA_OUT_PITCH 200
#define TIME_LORA_DATA_OUT 250

//#define LED_DEBUG
#define RPM_KHZ_OUTPUT
#define WHEEL_RPM_KHZ_OUTPUT

// Defines for the CAN IDs
#define GEAR_CAN_ID 1
#define PITCH_CAN_ID 2
#define MAST_DIR_CAN_ID 3
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

#define MAX_TURB_RPM_VALUE 400

// LEDs
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
//DigitalIn  test(PF_15);

// Threads
Thread ws_thread;
Thread acquisition_thread;
Thread data_out_thread;
Thread lora_data_out_thread;

// Analog inputs
AnalogIn loadcell_adc(A0);
AnalogIn torque_adc(A1);

// Weather Station input
Serial weather_station(PE_8, PE_7, 4800); // RX = PE_7, TX = PE_8
//BufferedSerial weather_station(PE_8, PE_7, 256);
Serial arduino_lora(PD_5, PD_6, 9600); // RX = PD_6, TX = PD_5d

// RPM
DigitalOut();
InterruptIn rpm_pin(PE_13);
InterruptIn wheel_rpm_pin(PE_11);

// Pitch position encoder
DigitalOut pitch_clock(PE_15);
DigitalIn pitch_input(PF_15);

// Flags
int flag_pc_out = false;

// structure to store sensor data
SensorData sensors;

float pitch_miclette_target = 0;

// LoRa
// Packet to send to PC over LoRa
LoraData lora_trame;
volatile bool lora_tx_successful = false;
// G0 interrupt for lora mode updates
//InterruptIn chip_interrupt(RFM95_INT);
//volatile bool isr_triggered = false;

bool CAN_transmit_status = false;

SensorData averaging_sensors[5];
uint32_t sensor_index = 0;

bool pitch_valid = false;
float pitch_target_algo = 0.0f;

bool volant_ROPS = false;

// Averaging the wind direction
//
#define WIND_ANGLE_HYS 15
#define WIND_ANGLE_SAMPLES 5

bool weather_station_up = false;
bool first_wind_dir = true;
float wind_directions[WIND_ANGLE_SAMPLES];
float wind_speeds[WIND_ANGLE_SAMPLES];
float wind_direction_avg = 0.0f;
float wind_speed_avg = 0.0f;
uint8_t wind_index = 0;

// Trames de la weather station
char trame[64];
int index_;
// RPM counter
uint32_t rpm_counter = 0;
uint32_t wheel_rpm_counter = 0;

//volatile uint32_t delta_steps = 0;

// Fonctions
void main_lora_transmission();
void main_acquisition();
void ws_acquisition();
void irq_rpm();
void irq_wheel_rpm();
void main_data_out();
void WriteDataToCAN();

// Pitch management functions
void send_pitch_command(float angle);
float pitch_management(float& current, float& target);
float read_target_angle_CAN();
float calc_pitch_angle(float vehicle_speed);
float send_pitch_delta(float pitch_delta);
float pitch_to_angle(float pitch_value);
float send_ROPS(bool ROPS);
void set_drive_pitch_mode(int mode);

void Isr0_impl();
void Isr0();

void CAN_recv(void);

// Union for sending 4 bytes of data, either has float, int or chars
union SendData
{
    SendData(float f) : data_f(f) {}
    SendData(int i) : data_i(i) {}

    float data_f;
    int data_i;
    char data[4];
};
void WriteDataToUART(char id, SendData data);
void WriteSensorsToUART();

// Enum for the UART data values
enum
{
  TORQUE_ID,
  LOADCELL_ID,
  TURB_RPM_ID,
  WHEEL_RPM_ID,
  WIND_SPEED_ID,
  WIND_DIRECTION_ID,
  PITCH_ID,
  LIMIT_SWITCH_ID,
  MAST_POSITION_ID,
  GEAR_ID,
  VOLTAGE_ID,
  CURRENT_ID,
  NUM_UART_VALUES
} stm32_data_values;

// Weather station thread
void ws_acquisition()
{
    while (true)
    {
        wait_ms(1); // Lessen the load on the uC
        if(weather_station.readable())
        {
            char character = weather_station.getc();

            if(character == '$' && index_ != 0)
            {
                // Process the last NMEA packet
                static char IIMWV_str[] = "$IIMWV";
                static char sentence_begin[7] = {0};
                memcpy(sentence_begin, trame, 6);
                if(!strcmp(IIMWV_str, sentence_begin))
                {
                    led1 = !led1;
                    weather_station_up = true;
                    // Extract information
                    static char wind_dir[6] = {0};
                    memcpy(wind_dir, trame + 7, 5);
                    static char wind_spd[6] = {0};
                    memcpy(wind_spd, trame + 15, 5);

                    sensors.wind_direction = atof(wind_dir);
                    sensors.wind_direction = (sensors.wind_direction > 180) ? sensors.wind_direction -360: sensors.wind_direction;
                    //sensors.wind_direction *= -1;
	                /*if (sensors.wind_direction < 0)
		                sensors.wind_direction = 180 - sensors.wind_direction;
	                else if (sensors.wind_direction >= 0)
		                sensors.wind_direction = -(180 + sensors.wind_direction);*/
                    float wind_speed = atof(wind_spd) * 0.514444;

                    // Discard packet if data is erroneous
                    if(wind_speed > 100)
                      continue;

                    sensors.wind_speed = wind_speed;


                    wind_directions[wind_index] = sensors.wind_direction;
                    wind_speeds[wind_index] = sensors.wind_speed;
                    wind_index = (wind_index + 1) % WIND_ANGLE_SAMPLES;

                    wind_direction_avg = (wind_directions[0] +
                                          wind_directions[1] +
                                          wind_directions[2] +
                                          wind_directions[3] +
                                          wind_directions[4]) / WIND_ANGLE_SAMPLES;
                    wind_speed_avg = (wind_speeds[0] +
                                      wind_speeds[1] +
                                      wind_speeds[2] +
                                      wind_speeds[3] +
                                      wind_speeds[4]) / WIND_ANGLE_SAMPLES;
                    if(first_wind_dir)
                    {
                      first_wind_dir = false;
                      wind_direction_avg = sensors.wind_direction;
                      wind_speed_avg = sensors.wind_speed;
                    }
                }
                // Clear the packet
                index_ = 0;
            }
            // Put the character read into our buffer
            trame[index_++] = character;
        }
    }
}

// LoRa communication management thread

void main_lora_transmission()
{
  while(1)
  {
    // Copy sensors data into Lora data packet
    memcpy(&lora_trame.sensors, &sensors, sizeof(SensorData));

    // Set other variables and flags
    //lora_trame.bROPS = false;
    //lora_trame.avg_wind_direction = 0.0f;
    //lora_trame.mast_mode = 0;
    //lora_trame.pitch_mode = 0;
    //lora_trame.errors = 0;
    //lora_trame.target_pitch = 0.0f;

    // Send data packet over lora
    bool status = lora::SendLoraData(lora_trame);
    lora_tx_successful = status;

    // Send only at a fixed time interval
    unsigned long starttime = us_ticker_read();
    while ((us_ticker_read() - starttime) < (TIME_LORA_DATA_OUT*1000))
    {
      // Active polling to check if ISR was triggered
      lora::CheckIsr0();

      wait_us(100);
    }
  }
}

// RPM IRQ handlers
void irq_rpm()
{
    ++rpm_counter;
}
void irq_wheel_rpm()
{
    ++wheel_rpm_counter;
}

// Performs the acquisition of the sensors
void main_acquisition()
{
    while(1)
    {
        // Torque
        sensors.torque = torque_adc.read() * 160.0;

        // Load Cell
        sensors.loadcell = loadcell_adc.read() * 500.0;
        //sensors.loadcell -= 457.0;

        // Rotor RPM
        //sensors.rpm_rotor = (float)rpm_counter * 1000.0f / (float)TIME_ACQ;
        sensors.rpm_rotor = rpm_counter;
//#ifndef RPM_KHZ_OUTPUT
        sensors.rpm_rotor *= 1000.0f/(float)TIME_ACQ * 60.0f / 360.0f;
//#endif
        rpm_counter = 0;

        // Wheel RPM
        static int cnt_wheel_rpm = 0;
        ++cnt_wheel_rpm;
        if(cnt_wheel_rpm >= 10)
        {
          //sensors.rpm_wheels = (float)wheel_rpm_counter * 1000.0f / (float)TIME_ACQ;
          sensors.rpm_wheels = wheel_rpm_counter;
  //#ifndef WHEEL_RPM_KHZ_OUTPUT
          sensors.rpm_wheels *= 1000.0f/(float)(TIME_ACQ * cnt_wheel_rpm) * 60.0f / 48.0f;
  //#endif
          wheel_rpm_counter = 0;
          cnt_wheel_rpm = 0;
        }

#ifdef LED_DEBUG
        led2 = !led2;
#endif

        wait_ms(1);

        // Pitch
        unsigned int pitch_data = 0;
        for(int i = 0; i < 22; ++i)
        {
            pitch_data <<= 1;
            pitch_clock = 0;
            wait_us(10);
            pitch_clock = 1;
            wait_us(10);
            pitch_data |= pitch_input;
        }

        // Check validity of pitch encoder
        if(pitch_data > 4194000)
        {
          // Assume that pitch is not valid
          pitch_valid = false;
        }
        else
        {
          pitch_valid = true;
          sensors.pitch = pitch_data;
        }

        wait_ms(TIME_ACQ-1);
    }
}

// Analyzes the data and sends it out
void main_data_out()
{
    while(1)
    {
#ifdef LED_DEBUG
        led3 = !led3;
#endif

        // Out PC
        // We cant call printf in an interrupt, so we flag it for main.
        flag_pc_out = true;

        // Out CAN
        WriteDataToCAN();

        // Out LoRa
        // Transmit serial UART to arduino for LoRa transmission
        WriteSensorsToUART();

        // Out SD
        // TODO

        wait_ms(TIME_DATA_OUT);
    }
}

void WriteDataToCAN()
{
    static int dummy_zero = 0;
    unsigned can_success = 1;

    // Dummy values to test the steering wheel dsiplays
    sensors.gear = 2;
    //sensors.pitch = PITCH_DRAPEAU;
    //sensors.rpm_rotor = 708.0f;
    //sensors.wind_speed = 11.5f;
    //sensors.current = 2.3;
    //sensors.voltage = 22.1;
    //sensors.rpm_wheels = pitch_target_algo;
    //sensors.wind_direction = 220.1f;
    //sensors.loadcell = 120.0f;
    //sensors.torque = 75.5f;
    //pc.printf("CAN\r\n");

    // Gear
    can_success &= can.write(CANMessage(GEAR_CAN_ID, (char*)&sensors.gear, 4));
    wait_us(200);
    // Pitch

    float pitch_angle = (3.0f / 2.0f) * pitch::pitch_to_angle((float)sensors.pitch);
    //pitch_angle = abs(pitch_angle);
    //pitch_angle = 15.6f;
    can_success &= can.write(CANMessage(PITCH_CAN_ID, (char*)&pitch_angle, 4));
    wait_us(200);

    // Mast dir + Mast mode
    // TODO
    // Calib done
    // TODO
    // Rotor RPM
    can_success &= can.write(CANMessage(ROTOR_RPM_CAN_ID, (char*)&sensors.rpm_rotor, 4));
    wait_us(200);
    // Wind Speed
    can_success &= can.write(CANMessage(WIND_SPEED_CAN_ID, (char*)&sensors.wind_speed, 4));
    wait_us(200);
    // Current + Voltage
    can_success &= can.write(CANMessage(CURRENT_CAN_ID, (char*)&dummy_zero, 4));
    wait_us(200);
    // Voltage
    can_success &= can.write(CANMessage(VOLTAGE_CAN_ID, (char*)&dummy_zero, 4));
    wait_us(200);
    // Wheel RPM
    //can_success &= can.write(CANMessage(WHEEL_RPM_CAN_ID, (char*)&sensors.rpm_wheels, 4));
    can_success &= can.write(CANMessage(WHEEL_RPM_CAN_ID, (char*)(&pitch_miclette_target), 4));
    wait_us(200);
    // Wind dir
    //unsigned int wind_dir = (unsigned int)(sensors.wind_direction);

    //can_success &= can.write(CANMessage(0x20, (char*)(&sensors.wind_direction), 4));
    wait_us(200);
    //wait_us(2000);

    // Acq Stat
    // TODO
    // Pitch Mode
    // TODO

    if(weather_station_up)
    {
      static int stop_mast_cmd = 0;
      static int last_sign = 0;

      // Timings
      static unsigned delta_time_us = 0;
      static unsigned last_time_us = us_ticker_read();
      unsigned current_time_us = us_ticker_read();
      delta_time_us += (current_time_us - last_time_us);
      last_time_us = current_time_us;

      float abs_wind = abs(sensors.wind_direction);
      float direction = (sensors.wind_direction >= 0) ? 1.0 : -1.0;
      float abs_wind_avg = abs(wind_direction_avg);
      unsigned change_dir = ((direction * last_sign) < 0) ? 1 : 0;

    	// Only send mast auto command if wind is above 2 knots
    	float pot_value = 0.0f;
    	if(wind_speed_avg > 2.0)
    	{
        if(change_dir || stop_mast_cmd)
        {
          stop_mast_cmd = 0;
          pot_value = 0.0;
        }
    		else if (abs_wind < 5.0 || abs_wind_avg < 5.0)
    		{
          pot_value = 0.0;
          // Check if 4s have passed

    		}
        else if(abs_wind_avg < 15.0)
        {
          //pot_value = 1.0 * direction;
          // Check if 4s have passed
          if(delta_time_us > 10000000)
          {
            pot_value = 1.0 * direction;
            delta_time_us = 0;
            //pot_value = 0.0;
            stop_mast_cmd = 1;
          }
        }
        //else if(abs_wind < 15.0)
        //{
        //
        //}
    		else
    		{
    			 pot_value = 15.0 * direction;
    		}
    	}
    	// Send control to the mast
    	//pot_value = 6.0;
    	can_success &= can.write(CANMessage(0x56, (char*)(&pot_value), 4));
    	wait_us(200);

      last_sign = direction;
    }

    // Loadcell
    can_success &= can.write(CANMessage(LOADCELL_CAN_ID, (char*)&sensors.loadcell, 4));
    wait_us(200);
    // Torque
    can_success &= can.write(CANMessage(TORQUE_CAN_ID, (char*)&sensors.torque, 4));
    wait_us(200);

    pc.printf("can success = %d\n\r", can_success);

    unsigned char errors = can.tderror();
    pc.printf("num write errors = %d\n\r", errors);
    if(errors)
    {
      //can.reset();
      CAN_transmit_status = false;
    }
    else
    {
        CAN_transmit_status = true;
    }
}

void WriteDataToUART(char id, SendData data)
{
    static uint8_t packet[6];
    packet[0] = '$';
    packet[1] = id;
    memcpy(&packet[2], &data, sizeof(SendData));
    for(int i = 0; i < 6; ++i)
    {
        arduino_lora.putc(packet[i]);
    }
}

void WriteSensorsToUART()
{
    WriteDataToUART(TORQUE_ID, SendData(sensors.torque));
    WriteDataToUART(LOADCELL_ID, SendData(sensors.loadcell));
    WriteDataToUART(TURB_RPM_ID, SendData(sensors.rpm_rotor));
    WriteDataToUART(WHEEL_RPM_ID, SendData(sensors.rpm_wheels));
    WriteDataToUART(WIND_SPEED_ID, SendData(sensors.wind_speed));
    WriteDataToUART(WIND_DIRECTION_ID, SendData(sensors.wind_direction));
    WriteDataToUART(PITCH_ID, SendData(sensors.pitch));
}

// main() runs in its own thread in the OS
int amain()
{
  can.frequency(250000);
  wait_ms(1);

  ws_thread.start(ws_acquisition);
  sensors.pitch = 999;

  while(1)
  {
    WriteDataToCAN();
    wait_ms(100);
  }
}

int main()
{
    //
    // Outputs and modules initialization
    //

    // CAN Init
    can.frequency(250000);

    // SPI Init
    // TODO : Init LoRa SPI

    // LoRa Init
    //bool lora_init_status = lora::Init();

    /// Set the interrupt Isr0 in the main thread
    ///chip_interrupt.rise(&Isr0);

    /*
    if(!lora_init_status)
    {
      pc.printf("LoRa Init Failed !!\n\r");
      while(1);
    }
    wait_ms(10);
    */

    //
    // Threads and callbacks
    //
    ws_thread.start(ws_acquisition);
    acquisition_thread.start(main_acquisition);
    data_out_thread.start(main_data_out);
    //lora_data_out_thread.start(main_lora_transmission);

    // RPM Interrupt on pin
    rpm_pin.rise(&irq_rpm);
    wheel_rpm_pin.rise(&irq_wheel_rpm);

    //
    // Init modes for drives
    //

    // Pitch drive mode
    wait_ms(100);
    pitch::SetMode(PITCH_AUTOMATIC);
    wait_ms(1);

    // Mast drive mode
    // TODO

    // Clear the screen string
    static char clear_str[] = "x[2Jx[H";
    clear_str[0] = clear_str[4] = 27; // ESC character replaces the x's

    //
    // Main Loop
    //

    while (true)
    {
        //
        // Perform security checks and validations
        //

        // ROPS
        // TODO : Implement and test ROPS based on turbine value
        //pitch::ROPS = false;
        if(sensors.rpm_rotor > MAX_TURB_RPM_VALUE)
        {
          pitch::ROPS = true;
        }
        if(volant_ROPS)
        {
          pitch::ROPS = true;
        }

        // Check if ROPS is on :
        if(pitch::ROPS)
        {
          pitch::SendROPSCmd((float)(sensors.pitch), true);
        }

        if(flag_pc_out)
        {
            flag_pc_out = false;

            // If ROPS is not set (perhaps reseted) then send ROPS false to the drive
            if(!pitch::ROPS)
            {
              pitch::SendROPSCmd((float)(sensors.pitch), false);
            }

            //
            // CAN polling
            //
            // TODO: Find a better place (and better way ?) for CAN
            static CANMessage msg;
            if(can.read(msg))
            {
              if(msg.id == 0x39)
              {
                //pc.printf("Received ROPS message\n\r");
                //pitch::ROPS = (char)(msg.data[0]);
                volant_ROPS = (char)(msg.data[0]);
                pitch::ROPS = false;
              }
              if(msg.id == 0x35)
              {
                pitch::pitch_done = true;
              }
            }

            // *** PITCH AUTO ***
            //
            // Automatic pitch is sent at the same rate as other outputs
            // This seems adequate for now, re-evaluate later
            //
            static float vehicle_speed = 0.0f;
            static float pitch_target = 0.0f;

            static int cnt_pitch_auto = 0;
            if(cnt_pitch_auto++ >= 5)
            {
              if(pitch_valid)
              {
                pitch_target = pitch::AutoPitchWheelRPM((float)sensors.pitch, sensors.rpm_wheels, vehicle_speed);
              }

              pitch_miclette_target = pitch_target;
              pitch_target_algo = pitch_target;
              cnt_pitch_auto = 0;
            }

            //
            // PC Serial output
            //

            // Clear the screen and reset the cursor pointer

            pc.printf(clear_str);

            // Print data to screen
            pc.printf("Torque = %f Nm\n\r", sensors.torque);
            pc.printf("Loadcell = %f lbs\n\r", sensors.loadcell);
            pc.printf("Rotor RPM = %f rpm\n\r", sensors.rpm_rotor);
            pc.printf("Wheel RPM = %f rpm\n\r", sensors.rpm_wheels);
            pc.printf("Wind direction = %f degs\n\r", sensors.wind_direction);
            pc.printf("AVG Wind direction = %f degs\n\r", wind_direction_avg);
            pc.printf("Wind speed = %f m/s\n\r", sensors.wind_speed);
            pc.printf("Pitch = %f   raw pitch = %d\n\r", (3.0f / 2.0f) * pitch::pitch_to_angle((float)sensors.pitch), sensors.pitch);
            //pc.printf("First time pitch steps = %f\n\r", delta_steps);
            //pc.printf("Current pitch for calc = %f\n\r", current_pitch);
            //pc.printf("First ever pitch = %f\n\r", first_pitch_value);
            pc.printf("Pitch target algo miclaye = %f\n\r", pitch_target);
            pc.printf("Vehicle speed = %f\n\r", vehicle_speed);
            //pc.printf("ROPS delta steps = %f\n\r", rops_steps);
            pc.printf("ROPS = %s  ,  volant ROPS = %s\n\r", (pitch::ROPS) ? "TRUE" : "FALSE", (volant_ROPS) ? "TRUE" : "FALSE");
            pc.printf("\n\r");
            pc.printf("Weather Station    -  %s\n\r", (weather_station_up) ? "ON" : "OFF");
            pc.printf("Encoder Pitch      -  %s\n\r", (pitch_valid) ? "ON" : "OFF");
            pc.printf("Wheel RPM          -  %s\n\r", (false) ? "ON" : "OFF");
            pc.printf("Turbine RPM        -  %s\n\r", (false) ? "ON" : "OFF");
            pc.printf("\n\r");
            pc.printf("LoRa status        -  %s\n\r", (lora_tx_successful) ? "SUCCESS" : "FAILURE");
            pc.printf("CAN status         -  %s\n\r", (CAN_transmit_status) ? "SUCCESS" : "FAILURE");
        }

        // Active looping at 5ms interval
        wait_ms(5);
    }
}

//
// Interrupts
//
void CAN_recv()
{
  CANMessage msg;
  if(can.read(msg))
  {
    if(msg.id == 0x39)
    {
      //ROPS = msg.data_i[0];
      pitch::ROPS = 1;
    }
    else if(msg.id == 0x35)
    {
      pitch::pitch_done = true;
    }
  }
}
