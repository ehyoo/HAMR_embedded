
#include "motor.h"
#include "Arduino.h"
#include "constants.h"

#define MAX_ROT_SPEED 180.0

void set_direction(int pin_driver_dir, bool dir) {
  digitalWrite(pin_driver_dir, dir);
}

// Sets direction and Arduino PWM to desired speed
void set_speed(PID_Vars* pid,
               float speed_req, 
               float speed_act,
               float* speed_cmd,
               float t_elapsed,
               int* pwm_val,
               int pin_driver_dir, 
               int pin_pwm, 
               float* pidError) {

  // increment speed_cmd with PID output (velocity control outputs change in speed)
  // float pid_output = pid->update_pid(speed_req, speed_act, t_elapsed);
  // *pwm_val = *pwm_val + round(255.0 * pid_output);
  // *pwm_val = constrain(*pwm_val, -255, 255); // limit PWM values
  
  //THIS
  // was +=
  *speed_cmd = pid->update_pid(speed_req, speed_act, t_elapsed);
  *pidError = speed_req - speed_act;
  
  // *pwm_val = round((speed_req)*255.0);
  
  
  
  // AND THIS
  *pwm_val = round((*speed_cmd)*255.0);
  *pwm_val = constrain(*pwm_val, -255, 255); // limit PWM values

   if (abs(*pwm_val) < 5) {
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


// THIS IS A MODIFIED VERSION OF THE CODE ABOVE.

void set_speed_of_turret(PID_Vars* pid,
               float speed_req, 
               float speed_act,
               float* speed_cmd,
               float t_elapsed,
               int* pwm_val,
               int pin_driver_dir, 
               int pin_pwm, 
               float* pidError) {

  // increment speed_cmd with PID output (velocity control outputs change in speed)
  // float pid_output = pid->update_pid(speed_req, speed_act, t_elapsed);
  // *pwm_val = *pwm_val + round(255.0 * pid_output);
  // *pwm_val = constrain(*pwm_val, -255, 255); // limit PWM values
  
  //THIS
  // was +=
  *speed_cmd = -1*pid->update_pid(speed_req, speed_act, t_elapsed);
  *pidError = speed_req - speed_act;
  
  // *pwm_val = round((speed_req)*255.0);
  // AND THIS
  *pwm_val = round((*speed_cmd)*255.0);
  *pwm_val = constrain(*pwm_val, -255, 255); // limit PWM values

   if (abs(*pwm_val) < 5) {
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

/*
 * Calculates speed from encoder counts over time
 * 
 * curr_count, prev_count: current and previous encoder counts
 * ticks_per_rev: number of encoder counts per revolution
 * dist_per_rev: circumference of wheel
 * time_elapsed: time in ms
 */
float get_speed(long encoder_counts,
                float ticks_per_rev, 
                float dist_per_rev, 
                float time_elapsed) {
  // Calculating the speed using encoder count
  /*Serial.print(encoder_counts);
  Serial.print(time_elapsed,2);
  Serial.print(ticks_per_rev,2);
  Serial.print(dist_per_rev,2);*/

  // if big number > 4000, and small number is <100, then do this:

  // otherwise do the original

  return ((((float) encoder_counts) / ticks_per_rev) * dist_per_rev) / (time_elapsed / 1000.0);
}

float get_speed_from_difference(long difference,
                float ticks_per_rev, 
                float dist_per_rev, 
                float time_elapsed) {
  // Calculating the speed using encoder count
  /*Serial.print(encoder_counts);
  Serial.print(time_elapsed,2);
  Serial.print(ticks_per_rev,2);
  Serial.print(dist_per_rev,2);*/

  // if big number > 4000, and small number is <100, then do this:

  // otherwise do the original

  return ((((float) difference) / ticks_per_rev) * dist_per_rev) / (time_elapsed / 1000.0);
}




/*
 * Calculates angular speed from encoder counts over time
 * 
 * encoder_counts: encoder counts over the time period time_elapsed
 * ticks_per_rev: number of encoder counts per revolution
 * time_elapsed: time in ms
 */
float get_ang_speed(long encoder_counts,
                    float ticks_per_rev,
                    float time_elapsed) {
  // Getting turret: -1 because spinning opposite direction of encoder.
  return 1 * 360 * (((float) encoder_counts) / ticks_per_rev) / (time_elapsed / 1000.0);}

  float get_ang_speed_from_difference(long difference,
                    float ticks_per_rev,
                    float time_elapsed) {
  return -1 * 360 * (((float) difference) / ticks_per_rev) / (time_elapsed / 1000.0);
  
  
  }
