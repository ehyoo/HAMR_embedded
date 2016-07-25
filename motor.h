#ifndef Motor_h
#define Motor_h

#include "pid.h"

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

void set_direction(int pin_driver_inA, int pin_driver_inB, bool dir);

void set_speed(PID_Vars* pid, 
               float speed_req, 
               float speed_act, 
               float* speed_cmd,
               float t_elapsed, 
               int* pwm_val,
               int pin_driver_dir,
               int pin_pwm,
               float* pidError);

void set_speed_of_turret(PID_Vars* pid, 
               float speed_req, 
               float speed_act, 
               float* speed_cmd,
               float t_elapsed, 
               int* pwm_val,
               int pin_driver_dir,
               int pin_pwm,
               float* pidError);

float get_speed_from_difference(long difference,
                float ticks_per_rev, 
                float dist_per_rev, 
                float time_elapsed);

float get_ang_speed_from_difference(long difference,
                    float ticks_per_rev,
                    float time_elapsed);

                    
#endif
