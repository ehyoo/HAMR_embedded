#include "localize.h"
#include "pid.h"
#include "Arduino.h"

/*
 * dtheta_req: angular velocity setpoint
 * dtheta_act: measured angular velocity from encoders
 * dtheta_cmd: value given to control law to change angle
 */
void angle_control(PID_Vars* pid, float dtheta_req, float dtheta_act, float* dtheta_cmd, float speed_req,
                   float* M1_speed, float* M2_speed, float wheel_dist, float wheel_rad, float t) {
/**
 * pid = dd_ctrl
 * dtheta_req = desired_dd_r
 * dtheta_act = hamr_loc.w
 * dtheta_cmd = dtheta_cmd
 * speed_req = desired_dd_v
 * m1_speed = desired_m1_v
 * m2_speed = desired_m2_v
 * wheel_dist = wheel_dist
 * wheel_rad = wheel_radius
 * t = t_elapsed
 * 
 */
  // USE FOR CONTROLLER INPUT: maps [-1,1]->[-PI,PI] rads
  float pid_output = pid->update_pid(dtheta_req * PI, dtheta_act, t); 
  *dtheta_cmd = pid_output;
  // Control law
  // Determine speeds for each indiv motor to achieve angle at speed
  float ang_speed = (wheel_dist/2.0) * (*dtheta_cmd);
  if (dtheta_req == 0.0) {
    // Remove any turning if not input in
    *M1_speed = -1 * speed_req;
    *M2_speed = speed_req;
  } else {
    //
    *M1_speed = -1 * (speed_req - ang_speed);
    *M2_speed = speed_req + ang_speed;
  }
}




