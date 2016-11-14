#ifndef DD_CONTROL_h
#define DD_CONTROL_h 


void angle_control(PID_Vars* pid, float dtheta_req, float dtheta_act, float* dtheta_cmd, float speed_req,
                   float* M1_speed, float* M2_speed, float wheel_dist, float wheel_rad, float t);

#endif


