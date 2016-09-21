#ifndef hamr_imu_h
#define hamr_imu_h

#include <stdint.h>

/*********************************
INITIALIZATION
**********************************/
void initialize_imu(); //takes in looptime in seconds


/*********************************
MAIN LOOP FUNCTION
**********************************/
void compute_imu(float _looptime);


/*********************************
GETTERS AND SETTERS
**********************************/
void set_x_y(float x, float y);

void set_angle(float degrees);

float get_current_x();

float get_current_y();

float get_current_angle();

float get_current_x_velocity();

float get_current_y_velocity();

float get_current_angular_velocity();

float get_angular_velocity_gyro_z();

void get_imu_raw(int* ax_, int* ay_, int* az_, int* gx_, int* gy_, int* gz_, int* mx_, int* my_, int* mz_);

/*********************************
PRINTING
**********************************/
void print_raw_imu();

void print_scaled_imu();

void print_calculated_linear();

void print_calculated_angular();

/*********************************
MISC
**********************************/
int is_imu_working();

float low_pass(float current, float filtered_prev, float alpha);

#endif

