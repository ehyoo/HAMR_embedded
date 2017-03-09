#include "holonomic_control.h"
#include "math.h"
#include "Arduino.h"
#include "constants.h"

#define DIM_A 0.1352065
#define DIM_B 0.1352065
#define DIM_R WHEEL_RADIUS

// this value will determine how quickly to change the setpoint
float max_linear_acceleration = .1;
float max_angular_acceleration = .1;

// desired user velocities
float setpoint_x = 0;
float setpoint_y = 0;
float setpoint_r = 0;

/* last known state (updated by update_holonomic_state()) */
float state_xdot = 0;
float state_ydot = 0;
float sensed_theta_d = 0; //theta_d is the angle of the drive relative to turret


// Actual velocities input into Jacobian. 
// These depend on the set_point and max_linear_acceleration
float desired_x = 0;
float desired_y = 0;
float desired_r = 0;

//last set of velocities output from jacobian 
float output_m1 = 0;
float output_m2 = 0;
float output_mt = 0;

 
// Update holonomic actuator velocities with given current known state. 
// Return the computed output in the output pointers
void get_holonomic_motor_velocities(float _state_theta_dheta_d, 
                                    float* _output_m1, 
                                    float* _output_m2,
                                    float* _output_mt) {
    sensed_theta_d = _state_theta_dheta_d;

    desired_x = setpoint_x;
    desired_y = setpoint_y;
    desired_r = setpoint_r;

    // compute jacobian based on input velocities. store outputs in the ouput_* variables
    compute_ramsis_jacobian(desired_x, desired_y, desired_r, sensed_theta_d);

    //compte_ramsis-jacobian updates the output variables
    * _output_m1 = -1 * output_m2; // to account for signs
    * _output_m2 = output_m1;
    * _output_mt = output_mt;
}

void compute_global_state(float sensed_m1, 
                        float sensed_m2,
                        float sensed_mt,
                        float sensed_t,
                        float* xdot,
                        float* ydot,
                        float* tdot){
    // Computes the current x, y, and t dot from given information
    float sint, cost;
    float b_s, b_c, a_s, a_c; //intermediate calculations
    float rac, rbc, ras, rbs;

    float converted_m1 = sensed_m1/WHEEL_RADIUS;
    float converted_m2 = sensed_m2/WHEEL_RADIUS;
    float converted_mt = sensed_mt * (PI/180);
 
    sint = sin(sensed_t);
    cost = cos(sensed_t);

    b_s = DIM_B * sint; 
    b_c = DIM_B * cost;
    a_s = DIM_A * sint;
    a_c = DIM_A * cost;

    rac = DIM_R * a_c;
    rbc = DIM_R * b_c;
    ras = DIM_R * a_s;
    rbs = DIM_R * b_s;

    // these pointers are: computed_xdot, computed_ydot and computed_tdot
    *xdot = ((-rbc - ras)*converted_m1 + (rbc - ras) * converted_m2) / (2.0 * DIM_A);
    *ydot = ((-rbs + rac)*converted_m1 + (rbs + rac) * converted_m2) / (2.0 * DIM_A);
    *tdot = (DIM_R * converted_m1 - DIM_R * converted_m2 )/ (2.0 * DIM_A) - converted_mt;
}


// Helper Function for update_holonomic_state()
// compute the jacobian with the given inputs
void compute_ramsis_jacobian(float desired_xdot,
                            float desired_ydot,
                            float desired_tdot,
                            float t){
    // Computed in radians....
    float sint, cost;
    float b_sin, b_cos, a_sin, a_cos; //intermediate calculations
    sint = sin(t);
    cost = cos(t);

    b_sin = DIM_B * sint;
    b_cos = DIM_B * cost;
    a_sin = DIM_A * sint;
    a_cos = DIM_A * cost;

    // jacobian
    output_m1 = (desired_xdot*(-b_sin - a_cos) + desired_ydot*(b_cos - a_sin)) / (DIM_B);
    output_m2 = (desired_xdot*(-b_sin + a_cos) + desired_ydot*(b_cos + a_sin)) / (DIM_B);
    // Then output as degrees. 

    //  Isolated Y works correctly, then X moves in correct direction but angles are flipped (90 at -x and 270 at +x) but reorients itself when sending y command
    //    output_mt = (180/PI) * ((-desired_xdot * cost - desired_ydot * sint) / DIM_B + desired_tdot);

    // Isolated Y works correctly, X angles are correct but moves in wrong direction and still flips itself around. Does not reorient itself after sending y command.
    output_mt   = (180/PI) * ((-desired_xdot * cost - desired_ydot * sint) / DIM_B + desired_tdot);
}

void set_holonomic_desired_velocities(float xdot, float ydot, float rdot) {
  // set in-file variables to commanded x, y, and r
    setpoint_x = xdot;
    setpoint_y = ydot;
    setpoint_r = rdot;
}

void set_max_linear_acceleration(float a){
    max_linear_acceleration = a;
}

void set_max_angular_acceleration(float a){
    max_angular_acceleration = a;
}



