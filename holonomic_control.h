#ifndef HOLO_CON_H
#define HOLO_CON_H

/*
Update holonomic actuator velocties with given current known state. 
Return the computed output in the output pointers
*/
void get_holonomic_motor_velocities(float _state_t, float* _output_m1, float* _output_m2, float* _output_mt);

/*
Given motor velocities, compute xdot, ydot, and thetadot in global system
*/
void compute_global_state(float sensed_m1, float sensed_m2, float sensed_mt,  float sensed_t, 
                          float* xdot, float* ydot, float* tdot);

/* 
Helper Function for update_holonomic_state()
compute the jacobian with the given inputs*/
void compute_ramsis_jacobian(float xdot, float ydot, float tdot, float t);

void set_holonomic_desired_velocities(float xdot, float ydot, float rdot);

void set_max_linear_acceleration(float a);

void set_max_angular_acceleration(float a);


#endif



