// motor.cpp
// deals with setting speed of motors as well as calculating speed from 
// encoder readings

// For debugging purposes, if you need to override the PID control and just do
// direct velocity control via voltage, comment lines labeled 1 and 2 in 
// set_speed and set_speed_of_turret and uncomment lines 3. 
#include "motor.h"
#include "Arduino.h"
#include "constants.h"

#define MAX_ROT_SPEED 180.0

void set_direction(int pin_driver_dir, bool dir) {
  digitalWrite(pin_driver_dir, dir);
}

void set_speed(PID_Vars* pid,
               float speed_req, 
               float speed_act,
               float* speed_cmd,
               float t_elapsed,
               int* pwm_val,
               int pin_driver_dir, 
               int pin_pwm, 
               float* pidError) {
    // Sets the direction and Arduino PWM of M1 and M2 to desired speed
    *speed_cmd = pid->update_pid(speed_req, speed_act, t_elapsed); // 1
    *pidError = speed_req - speed_act;
    // *pwm_val = round((speed_req)*255.0); //3
    *pwm_val = round((*speed_cmd)*255.0); // 2
    *pwm_val = constrain(*pwm_val, -255, 255); // limit PWM values

    if (abs(*pwm_val) < 5) {
        // Deadband to help with nosie from encoders
        *pwm_val = 0.0;
    }

    if (*pwm_val < 0) {
        // reverse direction
        set_direction(pin_driver_dir, !M1_FORWARD);
    } else {
        // forward direction
        set_direction(pin_driver_dir, M1_FORWARD);
    }
    analogWrite(pin_pwm, abs(*pwm_val));
} 

void set_speed_of_turret(PID_Vars* pid,
                        float speed_req, 
                        float speed_act,
                        float* speed_cmd,
                        float t_elapsed,
                        int* pwm_val,
                        int pin_driver_dir, 
                        int pin_pwm, 
                        float* pidError) {
    // Sets speed of turret
    // Only thing that's different between the above function and this is the 
    // -1 in *speed_cmd, which is to take into consideration the differing
    // directions of the motor controlling the turret and the turret itself.
    *speed_cmd = -1 * pid->update_pid(speed_req, speed_act, t_elapsed); // 1
    *pidError = speed_req - speed_act;
    // *pwm_val = round((speed_req)*255.0); // 3
    *pwm_val = round((*speed_cmd)*255.0); //2
    *pwm_val = constrain(*pwm_val, -255, 255); // limit PWM values

    if (abs(*pwm_val) < 5) {
    // Deadband
     *pwm_val = 0.0;
    }
  
    if (*pwm_val < 0) {
        // reverse direction
        set_direction(pin_driver_dir, !M1_FORWARD);
    } else {
        // forward direction
        set_direction(pin_driver_dir, M1_FORWARD);
    }
    analogWrite(pin_pwm, abs(*pwm_val));
} 


float get_speed_from_difference(long difference,
                                float ticks_per_rev, 
                                float dist_per_rev, 
                                float time_elapsed) {
    // Calculates speed from encoder counts
    // difference: difference between previous and current encoder counts
    // ticks_per_rev: number of encoder counts per revolution
    // time_elapsed: time in milliseconds
    return ((((float) difference) / ticks_per_rev) * dist_per_rev) / (time_elapsed / 1000.0);
}

float get_ang_speed_from_difference(long difference,
                  float ticks_per_rev,
                  float time_elapsed) {
    // Calculates angular speed of encoder in degrees/s
    // difference: difference between prev and current encoder counts
    // ticks_per_rev: number of encoder counts per revolution
    // time_elapsed: time in milliseconds
    return -1 * 360 * (((float) difference) / ticks_per_rev) / (time_elapsed / 1000.0);
}
