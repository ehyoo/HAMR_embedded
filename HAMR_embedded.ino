/***********************************/
/*                                 */
/*        HAMR_embedded.ino        */
/* Version 3 of HAMR embedded code */
/*             MODLAB              */
/*                                 */
/***********************************/
// Notes:
// M1 => RIGHT MOTOR
// M2 => LEFT MOTOR
// MT => TURRET MOTOR
#include "pid.h"
#include "motor.h"
#include "localize.h"
#include "hamr_imu.h"
#include "dd_control.h"
#include "constants.h"
#include "holonomic_control.h"
#include <Wire.h> // I2C 
#include <libas.h> // SSI Communication

/*******************************/
/*   ROS Serial Communication  */
/*******************************/
// Message Importing
#include <ros.h>
#include <hamr_interface/HamrStatus.h>
#include <hamr_interface/MotorStatus.h>
#include <hamr_interface/HamrCommand.h>
#include <hamr_interface/HoloStatus.h>
#include <hamr_interface/VelocityStatus.h>
#include <ros/time.h>

// Node and Publishing
ros::NodeHandle nh;
hamr_interface::HamrStatus hamrStatus;
hamr_interface::MotorStatus leftMotor;
hamr_interface::MotorStatus rightMotor;
hamr_interface::MotorStatus turretMotor;
// By default, should publish to hamr_state topic
// ros::Publisher pub("hamr_state", &hamrStatus);

// Holonomic debugging 
hamr_interface::HoloStatus holoStatus;
ros::Publisher pub("holo_state", &holoStatus);

// Turret Velocity debugging messages
// hamr_interface::VelocityStatus velStatus;
// ros::Publisher pub("vel_state", &velStatus);
  
// Subscribing
void command_callback(const hamr_interface::HamrCommand& command_msg);
ros::Subscriber<hamr_interface::HamrCommand> sub("hamr_command", &command_callback);

/***************************************************/
/*                                                 */
/*                   VARIABLES                     */
/*                                                 */
/***************************************************/
int debugmessage = 0;
/******************/
/* Desired Values */
/******************/
// holonomic velocities
float desired_h_xdot = 0; 
float desired_h_ydot = 0; 
float desired_h_rdot = 0;

// differential drive velocities
float desired_dd_v = 0; // Diff Drive (m/s)
float desired_dd_r = 0; // desired angular velocity for Diff Drive: set between [-1,1] by controller, mapped to [-90,90] degrees in code
// float speed_req_turret = 0.0; // Turret (rad/s)?

// motor velocities
float desired_M1_v = 0; 
float desired_M2_v = 0; 
float desired_MT_v = 0;

/*****************/
/* Sensed Values */
/*****************/
// sensed motor velocities at a specific point in time
float sensed_M1_v = 0.0;
float sensed_M2_v = 0.0;
float sensed_MT_v = 0.0;

// Previously sensed velocities
// Used in the current implementation of the low-pass filter
float sensed_M1_v_prev = 0.0;
float sensed_M2_v_prev = 0.0;
float sensed_MT_v_prev = 0.0;

// Velocity after the sensed_M[x]_v is passed through the filter
// NOTE: This should be the velocity used for all calculations
float sensed_M1_v_filt = 0.0;
float sensed_M2_v_filt = 0.0;
float sensed_MT_v_filt = 0.0;

/************************/
/*    Computed Values   */
/************************/
// Computed holonomic velocities
float computed_xdot = 0.0;
float computed_ydot = 0.0;
float computed_tdot = 0.0;

// Summation of turret 
// TODO: This will eventually overflow after a long time #
// This is a corner case that shouldn't happen in normal operation, but still.
long decoder_turret_total = 0;

/******************/
/* Set Speed Vars */
/******************/
// NOTE: I don't see why these should be vars in the main file
// motor PWMs
int pwm_M1 = 0;
int pwm_M2 = 0;
int pwm_MT = 0;
float M1_v_cmd = 0;
float M2_v_cmd = 0;
float MT_v_cmd = 0;

/**********************/
/* Control Parameters */
/**********************/
// Low-level PID 
PID_Vars pid_vars_M1(0.4, 8.0, 0.005);
PID_Vars pid_vars_M2(0.4, 8.0, 0.005);
PID_Vars pid_vars_MT(0.005, 0.07, 0.0);
PID_Vars dd_ctrl(0.1, 0.0, 0.0);

// Holonomic PID- NOT BEING USED
PID_Vars pid_vars_h_xdot(0.1, 0.0, 0.0);
PID_Vars pid_vars_h_ydot(0.1, 0.0, 0.0);
PID_Vars pid_vars_h_rdot(0.001, 0.0, 0.0);

// Velocity control command
// NOTE: These vars could be deleted provided we're not doing PID stuff
float h_xdot_cmd = 0; 
float h_ydot_cmd = 0; 
float h_rdot_cmd = 0; //holonomic
float dtheta_cmd = 0; //differential drive

/**********************/
/*       Sensor       */
/**********************/
// decoder counts
// M1 and M2 are between 0 and 4095
// MT is between 0 and 1093
int decoder_count_M1 = 0;
int decoder_count_M2 = 0;
int decoder_count_MT = 0;

// Size of the array used for the lowpass filter
// If the array low pass filter is commented out (which it usually is)
// this should not be used
const int AVG_FILT_SZ = 5;

// Arrays for said lowpass filters
float decoder_count_arr_M1[AVG_FILT_SZ];
float decoder_count_arr_M2[AVG_FILT_SZ];
float decoder_count_arr_MT[10]; // arbitrarily set to 10

// Previously recorded motor readings- used to find velocity
int decoder_count_M1_prev = 0;
int decoder_count_M2_prev = 0;
int decoder_count_MT_prev = 0;

/**********************/
/*    IMU Settings    */
/**********************/
// This is not used because we don't have an IMU
const float SENSOR_LOOPTIME = 10; //10000 //remeber to update this to microseconds later, or an exact timing method if possible
unsigned long next_sensor_time = millis(); // micros()
unsigned long prev_micros = 0;
float sensed_drive_angle = 0;

/************************/
/*        Timing        */
/************************/
unsigned long start_time; // Time at which the arduino starts
unsigned long last_recorded_time = 0;
float time_elapsed; // start_time - last_recorded_time
int loop_time_duration; // Timing loop time- for performance testing

/******************************/
/*    Test Drive Variables    */
/******************************/
unsigned long start_test_time;
unsigned long current_time;
float seconds = 0;
float R = 0;
bool square_test_did_start = false;
bool right_test_did_start = false;
bool circle_test_did_start = false;
bool spiral_test_did_start = false;
bool sine_test_did_start = false;
bool heading_circle_test_did_start = false;
bool timer_set = false;

/***************************/
/*          Modes          */
/***************************/
// Use direct drive by sending false to both dif_drive and holonomic_drive
bool use_dif_drive = false; // Sending linear velocity commands and turret commands
bool use_holonomic_drive = true; // Full fledged holonomic drive

/****************************************/
/*          Motor Calculations          */
/****************************************/
// Previously sensed velocities
float prev_sensed_velocity_right;
float prev_sensed_velocity_left;
float prev_sensed_velocity_turret;
// Instantiated objects for SSI communication using the libas library
libas libas_M1(30, 32, 28, 12);
libas libas_M2(24, 26, 22, 12);
libas libas_MT(38, 34, 36, 10);

/***********************/
/*    Miscellaneous    */
/***********************/
// Initial angle offset 
float offset = 0;
bool did_set_offset = false;

// dd localization
location hamr_loc; // This isn't being used right now. 
// Debugging vars- we should look to delete these
float pidError; // PID debugging for turret. See pid.h
float dummy1 = 0;
float dummy2 = 0;

/***************************************************/
/*                                                 */
/*                      SETUP                      */
/*                                                 */
/***************************************************/
void setup() {
    nh.initNode();              // Initialize ros node
    nh.subscribe(sub);          // arduino node subscribes to topic declared
    nh.advertise(pub);          // advertise the topic 
    // pinMode(40, OUTPUT);     // LED Debugging purposes
    init_actuators();           // initialiaze all motors
    //init_I2C();               // initialize I2C bus as master
    start_time = millis();      // Start timer
}

/***************************************************/
/*                                                 */
/*                      MAIN                       */
/*                                                 */
/***************************************************/
void loop() {
    while (1) {
        loop_time_duration = millis() - last_recorded_time;
        if ((millis() - last_recorded_time) >= LOOPTIME) { // ensures stable loop time
            time_elapsed = (float) (millis() - last_recorded_time);
            last_recorded_time = millis();
            compute_sensed_motor_velocities(); // read encoders
            calculate_sensed_drive_angle();
            send_serial(); // Send the current state via ROS
            check_for_test_execution(); // takes care of drive demo test commands.TODO prevent this from running test if kill command was sent

            if (use_dif_drive) {
                // NOTE: Differential drive is broken currently-
                // giving T command, the whole body moves instead of the turret 
                // TODO: Fix isolated differential drive
                differential_drive();
            } else if (use_holonomic_drive) {
                holonomic_drive();
            }
            set_speed_of_motors();
        }

        /* IMU Code */
        // Since we don't have an IMU the code remains as it is from the original

        //analogWrite(MT_PWM_PIN, 50);
        // update_prevs();
        //
        //    if (next_sensor_time < micros() && is_imu_working()) {
        //      unsigned long current_micros = micros();
        //
        //      compute_imu((current_micros - prev_micros) / 1000000.0); //update imu with time change
        //
        //      sensed_drive_angle = get_current_angle() * PI / 180;
        //
        //      next_sensor_time = micros() + SENSOR_LOOPTIME;
        //      prev_micros = current_micros;
        //
        //      // potentially combine hamr_loc.theta with imu angle?
        //    } else if (!is_imu_working()) {
        //      sensed_drive_angle = hamr_loc.theta;
        //    }
    }
}

/************************/
/*   Driving Functions  */
/************************/
void differential_drive() {
    // takes care of differential drive
    int use_dd_control = 1;
    if (use_dd_control == 0) {
        // PID velocity control, same input to both motors
        desired_M1_v = desired_dd_v;
        desired_M2_v = desired_dd_v;
    } else if (use_dd_control == 1) {
        // Differential drive control
        angle_control(&dd_ctrl, desired_dd_r, hamr_loc.w, &dtheta_cmd, desired_dd_v, &desired_M1_v, &desired_M2_v, WHEEL_DIST, WHEEL_RADIUS, time_elapsed);
    } else {
        // use indiv setpoints
        desired_M1_v = (desired_dd_v - (WHEEL_DIST/2.0) * PI/2.0);
        desired_M2_v = (desired_dd_v + (WHEEL_DIST/2.0) * PI/2.0);
    }
}

void holonomic_drive() {
    // Takes care of holonomic drive
    // Gets the current x, y, and t dot then uses those values to calculate
    // jacobian and adjusts the motors.
    compute_global_state(-1 * sensed_M1_v, //outputs x,y,r position of system. Could be used to close the loop. Not used for anything-could be unnecessary computing time
                        sensed_M2_v,
                        sensed_MT_v,
                        2*PI*sensed_drive_angle,
                        &computed_xdot,
                        &computed_ydot,
                        &computed_tdot);
    h_xdot_cmd = desired_h_xdot;
    h_ydot_cmd = desired_h_ydot;
    h_rdot_cmd = desired_h_rdot;

        // UNCOMMENT THE FOLLOWING LINE TO ENABLE HOLONOMIC PID
    // h_xdot_cmd = pid_vars_h_xdot.update_pid(desired_h_xdot, computed_xdot, time_elapsed);
    // h_ydot_cmd = pid_vars_h_ydot.update_pid(desired_h_ydot, computed_ydot, time_elapsed);
    // h_rdot_cmd = pid_vars_h_rdot.update_pid(desired_h_rdot, computed_tdot * 180 / PI, time_elapsed);

    // using output of holonomic PID, compute jacobian values for motor inputs
    set_holonomic_desired_velocities(h_xdot_cmd, h_ydot_cmd, h_rdot_cmd); // set these setpoints to the output of the holonomic PID controllers
    get_holonomic_motor_velocities(sensed_drive_angle * 2 * PI, &desired_M1_v, &desired_M2_v, &desired_MT_v);
    // get_holonomic_motor_velocities(hamr_loc.theta, &desired_M1_v, &desired_M2_v, &desired_MT_v);
}

void calculate_sensed_drive_angle() {
    // Gets the sensed drive angle of the current turret
    // When the HAMR first begins, it sets the current sensed angle as the
    // arbitrary 0.
    float ticks = TICKS_PER_REV_TURRET;
    sensed_drive_angle = fmod(decoder_turret_total, ticks) / (float) ticks;
    if (sensed_drive_angle < 0) {
        sensed_drive_angle = 1 + sensed_drive_angle;
    } 
    if (!did_set_offset) {
        offset = sensed_drive_angle;
        did_set_offset = true;
    }
    sensed_drive_angle = sensed_drive_angle - offset;
}

void set_speed_of_motors() {
    // sets the speed of all three of the motors
    set_speed(&pid_vars_M1,
            desired_M1_v,
            sensed_M1_v,
            &M1_v_cmd,
            time_elapsed,
            &pwm_M1,
            M1_DIR_PIN,
            M1_PWM_PIN,
            &dummy1);

    set_speed(&pid_vars_M2,
            desired_M2_v,
            sensed_M2_v,
            &M2_v_cmd,
            time_elapsed,
            &pwm_M2,
            M2_DIR_PIN,
            M2_PWM_PIN,
            &dummy2);

    set_speed_of_turret(&pid_vars_MT,
                        desired_MT_v,
                        sensed_MT_v,
                        &MT_v_cmd,
                        time_elapsed,
                        &pwm_MT,
                        MT_DIR_PIN,
                        MT_PWM_PIN,
                        &pidError);
}

/***********************/
/*    ROS Functions    */
/***********************/

void command_callback(const hamr_interface::HamrCommand& command_msg) {
    // called when message is sent to arduino
    // matches the message type with each case and does its respective routine
    // More often than not simply reassigning a variable.

    // the HamrCommand msg is detailed as follows:
    // string type (the type that corresponds to the switch cases)
    // string val (the value of the float)
    String str;
    float temp;
    float* sig_var;
    char type = static_cast<char>(command_msg.type);
    String val = command_msg.val;
    debugmessage = type;
    switch (type) {
      // holonomic inputs
        case SIG_HOLO_X:
            sig_var = &desired_h_xdot;
            break;

        case SIG_HOLO_Y:
            sig_var = &desired_h_ydot;
            break;

        case SIG_HOLO_R:
            sig_var = &desired_h_rdot;
            break;

      // differential drive inputs
        case SIG_DD_V:
            sig_var = &desired_dd_v;
            break;

        case SIG_DD_R:
            sig_var = &desired_dd_r;
            break;

      // motor velocities
        case SIG_R_MOTOR:
            sig_var = &desired_M1_v;
            break;

        case SIG_L_MOTOR:
            sig_var = &desired_M2_v;
            break;

        case SIG_T_MOTOR:
            sig_var = &desired_MT_v;
            break;

      // right motor PID
        case SIG_R_KP:
            sig_var = &(pid_vars_M1.Kp);
            break;

        case SIG_R_KI:
            sig_var = &(pid_vars_M1.Ki);
            break;

        case SIG_R_KD:
            sig_var = &(pid_vars_M1.Kd);
            break;

      // left motor PID
        case SIG_L_KP:
            sig_var = &(pid_vars_M2.Kp);
            break;

        case SIG_L_KI:
            sig_var = &(pid_vars_M2.Ki);
            break;

        case SIG_L_KD:
            sig_var = &(pid_vars_M2.Kd);
            break;

      // turret motor PID
        case SIG_T_KP:
            // sig_var = &(pid_vars_MT.Kp); 
            sig_var = &(dd_ctrl.Kp);
        break;

        case SIG_T_KI:
            // sig_var = &(pid_vars_MT.Ki);
            sig_var = &(dd_ctrl.Ki);
        break;

        case SIG_T_KD:
            // sig_var = &(pid_vars_MT.Kd);
            sig_var = &(dd_ctrl.Kd);
            break;

      // holonomic X PID
        case SIG_HOLO_X_KP:
            sig_var = &(pid_vars_h_xdot.Kp);
            break;

        case SIG_HOLO_X_KI:
            sig_var = &(pid_vars_h_xdot.Ki);
            break;

        case SIG_HOLO_X_KD:
            sig_var = &(pid_vars_h_xdot.Kd);
            break;

      // holonomic Y PID

        case SIG_HOLO_Y_KP:
            sig_var = &(pid_vars_h_ydot.Kp);
            break;

        case SIG_HOLO_Y_KI:
            sig_var = &(pid_vars_h_ydot.Ki);
            break;

        case SIG_HOLO_Y_KD:
            sig_var = &(pid_vars_h_ydot.Kd);
            break;

      // holonomic R PID

        case SIG_HOLO_R_KP:
            sig_var = &(pid_vars_h_rdot.Kp);
            break;

        case SIG_HOLO_R_KI:
            sig_var = &(pid_vars_h_rdot.Ki);
            break;

        case SIG_HOLO_R_KD:
            sig_var = &(pid_vars_h_rdot.Kd);
            break;
      // Tests on command
        case -100:
            // Square Test
            square_test_did_start = true;
            break;

        case -101:
            // Right Angle Test
            right_test_did_start = true;
            break;

         case -102:
         //Circle Test
         circle_test_did_start = true;
         break;

        case -103:
        //Spiral Test
        spiral_test_did_start = true;
        break;

        case -104:
        //Sinusoid Test
        sine_test_did_start = true;
        break;

        case -105:
        //Heading Circle
        heading_circle_test_did_start = true;
        break;
    }
      *sig_var = val.toFloat();
}

int turret_tick_change; // For debugging purposes for sending through serial- you should delete this later
  
void send_serial() {
   //leftMotor.position = decoder_count_M2;
   //rightMotor.position = decoder_count_M1;
   //turretMotor.position = decoder_count_MT;
   // Arbitrary 1000 multiplied to ensure that we can send it over as an int. 
   // Just think of it as mm/s 
   // This should be fixed soon.
   //leftMotor.velocity = (int)(sensed_M2_v * 1000);
   //rightMotor.velocity = (int)(sensed_M1_v * 1000);
   //turretMotor.velocity = (int)(sensed_MT_v * 100);
   //leftMotor.desired_velocity = (int)(desired_M2_v * 1000);
   //rightMotor.desired_velocity = (int)(desired_M1_v * 1000);
   //turretMotor.desired_velocity = (int)(desired_MT_v * 100);
   // These should be deleted later- these were put into messages purely for debugging
   //turretMotor.speed_cmd = (int)(roundf(MT_v_cmd ));
//   leftMotor.speed_cmd = 0;
//   rightMotor.speed_cmd = 0;
   //leftMotor.speed_cmd = (int)(roundf(M2_v_cmd ));
   //rightMotor.speed_cmd = (int)(roundf(M1_v_cmd ));
   
   //turretMotor.pidError = (int)(roundf(pidError * 100));
   //leftMotor.pidError = 0;
   //rightMotor.pidError = 0;
//   hamrStatus.timestamp = nh.now();
//   hamrStatus.left_motor = leftMotor;
//   hamrStatus.right_motor = rightMotor;
//   hamrStatus.turret_motor = turretMotor;
//   hamrStatus.looptime = loop_time_duration;
//   pub.publish(&hamrStatus);

//    holoStatus.setpoint_x =  (int)(h_xdot_cmd * 1000);
    holoStatus.setpoint_y = (int)(h_ydot_cmd * 1000);
    holoStatus.setpoint_r = (int)(h_rdot_cmd * 1000);
    holoStatus.setpoint_x = seconds * 1000;
    holoStatus.xdot = (int)(computed_xdot*1000);
    holoStatus.ydot = (int)(computed_ydot*1000);
    holoStatus.tdot = (int)(computed_tdot*100);
    holoStatus.left_vel = (int)(sensed_M2_v * 1000);
    holoStatus.right_vel = (int)(sensed_M1_v * 1000);
    holoStatus.turret_vel = (int)(sensed_MT_v * 100);
    holoStatus.desired_left_vel = (int) (desired_M2_v * 1000);
    holoStatus.desired_right_vel = (int) (desired_M1_v * 1000);
    holoStatus.desired_turret_vel = (int) (desired_MT_v * 100);
    holoStatus.sensed_drive_angle = (int)(sensed_drive_angle*360);
    pub.publish(&holoStatus);


    // turret velocity debugging things
//    velStatus.sensed_t_motor_enc_value = decoder_count_MT;
//    velStatus.sensed_t_motor_velocity = (int) (((float(turret_tick_change)/1023))/(time_elapsed/1000) * 1000);
//    velStatus.sensed_turret_position = (int) (360 * sensed_drive_angle);
//    velStatus.sensed_turret_velocity = (int) (sensed_MT_v * 100);
//    velStatus.desired_turret_velocity = (int) (desired_MT_v * 100);
//    velStatus.pid_error = (int)(roundf(pidError * 100));  
//    pub.publish(&velStatus);
    
    nh.spinOnce();
    
}

/******************************************************/
/*      Encoder reading and Velocity Calculations     */
/******************************************************/
void init_actuators() {
    // Set DD motor driver pins as outputs
    pinMode(M1_DIR_PIN, OUTPUT);
    pinMode(M2_DIR_PIN, OUTPUT);
    pinMode(MT_DIR_PIN, OUTPUT);

    // Set Motors as forward
    digitalWrite(M1_DIR_PIN, M1_FORWARD); // LOW is forwards
    digitalWrite(M2_DIR_PIN, M2_FORWARD); // HIGH is forwards
    digitalWrite(MT_DIR_PIN, MT_COUNTER); // HIGH is CLOCKWISE (use low for default)

    // Initialize PWMs to 0
    analogWrite(M1_PWM_PIN, 0);
    analogWrite(M2_PWM_PIN, 0);
    analogWrite(MT_PWM_PIN, 0);
}

void compute_sensed_motor_velocities() {
    // Gets the current position, calculates velocity from current and previous
    
    // Read encoders via libas 
    decoder_count_M1 = libas_M1.GetPosition();
    decoder_count_M2 = libas_M2.GetPosition();
    decoder_count_MT = libas_MT.GetPosition();

    // Calculating the difference between prev and current sensed positions
    float decoder_count_change_M1 = calculate_decoder_count_change(decoder_count_M1_prev, decoder_count_M1, 4095, 500, 3500);
    float decoder_count_change_M2 = calculate_decoder_count_change(decoder_count_M2_prev, decoder_count_M2, 4095, 500, 3500);
    float decoder_count_change_MT = calculate_decoder_count_change(decoder_count_MT_prev, decoder_count_MT, 1023, 300, 700);

    // Puts change into the total so we know the drive angle of turret
    decoder_turret_total -= decoder_count_change_MT;
    // turret_tick_change = decoder_count_change_MT; // For debugging purposes, see send_serial above

    decoder_count_M1_prev = decoder_count_M1;
    decoder_count_M2_prev = decoder_count_M2;
    decoder_count_MT_prev = decoder_count_MT;

    // Moving average filter on decoder count differences
    // Comment this back in if you want this kind of filter
    // for (int i = 1; i < AVG_FILT_SZ; i++) {
    //     decoder_count_arr_M1[i] = decoder_count_arr_M1[i - 1];
    //     decoder_count_arr_M2[i] = decoder_count_arr_M2[i - 1];
    // }
    // for (int j = 1; j < 10; j++) {
    //     decoder_count_arr_MT[j] = decoder_count_arr_MT[j - 1];
    // }
    // decoder_count_arr_M1[0] = decoder_count_change_M1;
    // decoder_count_arr_M2[0] = decoder_count_change_M2;
    // decoder_count_arr_MT[0] = decoder_count_change_MT;
    // float decoder_count_change_filt_M1 = compute_avg(decoder_count_arr_M1, AVG_FILT_SZ);
    // float decoder_count_change_filt_M2 = compute_avg(decoder_count_arr_M2, AVG_FILT_SZ);
    // float decoder_count_change_filt_MT = compute_avg(decoder_count_arr_MT, AVG_FILT_SZ);

    // // compute robot velocities
    // sensed_M1_v = get_speed(decoder_count_change_filt_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed);
    // sensed_M2_v = get_speed(decoder_count_change_filt_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed);
    // // sensed_MT_v = get_speed(decoder_count_change_filt_MT, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed);
    // sensed_MT_v = get_ang_speed(decoder_count_change_filt_MT, TICKS_PER_REV_TURRET, time_elapsed);

    // Get current velocities from differences
    float current_vel_right = get_speed_from_difference(decoder_count_change_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed);
    float current_vel_left = get_speed_from_difference(decoder_count_change_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed);
    float current_vel_turret = get_ang_speed_from_difference(decoder_count_change_MT, TICKS_PER_REV_TURRET, time_elapsed);
    // Then pass velocities to filter to get 'true' velocity
    // M1 and M2 returns m/s
    // MT returns degrees/s
    sensed_M1_v = low_pass_velocity_filter(current_vel_right, prev_sensed_velocity_right);
    sensed_M2_v = low_pass_velocity_filter(current_vel_left, prev_sensed_velocity_left);
    sensed_MT_v = low_pass_velocity_filter(current_vel_turret, prev_sensed_velocity_turret);

    // then assign current velocity to previous
    prev_sensed_velocity_right = current_vel_right;
    prev_sensed_velocity_left = current_vel_left;
    prev_sensed_velocity_turret = current_vel_turret;

    // Something about position but we don't use this
    // It also takes ~400 milliseconds to execute. 
    // hamr_loc.update(sensed_M1_v, sensed_M2_v, WHEEL_DIST, time_elapsed);
}

float low_pass_velocity_filter(float current, float prev) {
    // Simple low pass filter
    float beta = 0.386; // Calculated 0.386, but 0.6 works well
    return beta * current + (1 - beta) * prev; //filter at 10 hz - Tarik.
}

float compute_avg(float* arr, int sz) {
    // Compute average of some array
    // Used in previous implementation of low-pass filter
    float sum = 0;
    for (int i = 0; i < sz; i++) {
        sum += arr[i];
    }
    return sum / (float) sz;
}

float calculate_decoder_count_change(
    int prev, int current, int max, int lim_min, int lim_max) {
    /*
    prev: previously sensed
    current: currently sensed
    max: encoder max reading
    lim_min and lim_max: desired floor and ceiling at which you determine that 
    there has been an overflow.

    Calculates the difference of ticks between previously sensed and currently 
    sensed. Takes into consideration if there has been an overflow between
    readings (ie, encoder reads 4090 at time t, moves forward, then reads
    3 at time t+1)
    */
    if (prev > lim_max && current < lim_min) {
        return max - prev + current;
    } else if (prev < lim_min && current > lim_max) {
        return -1 * (max - current + prev);
    } else {
        return current - prev;
    }
}

/******************/
/*    I2C Code    */
/******************/
// Hamr V3 does not have I2C implementation yet. 
void init_I2C() {
  Wire.begin();
}

void request_decoder_count(char slave_addr) {
  Wire.beginTransmission(slave_addr);
  int bytes_available = Wire.requestFrom(slave_addr, (uint8_t) 4);

  if (bytes_available == 4)
  {
    decoder_count_M1 = Wire.read() << 8 | Wire.read();
    decoder_count_M2 = Wire.read() << 8 | Wire.read();
    // decoder_count_MT = Wire.read() << 8 | Wire.read();

    Serial.print("count0: "); Serial.print(decoder_count_M1);
    Serial.print(" count1: "); Serial.println(decoder_count_M2);
    // Serial.print(" count2: "); Serial.println(decoder_count_MT);

  }
  else
  {
    Serial.println("I2C error. Bytes Received: "); Serial.println(bytes_available);
    // light up an LED here
  }
  Wire.endTransmission();
}

void test_I2C_decoder_count() {
  request_decoder_count(99);
  delayMicroseconds(10000);
}

/***************************************************/
/*                                                 */
/*              Holonomic Drive Tests              */
/*                                                 */
/***************************************************/
void check_for_test_execution() {
  // checks if the HAMR should start to execute any tests
    if (square_test_did_start) {
        
        if (!timer_set) {
            start_test_time = millis();
            timer_set = true;
        }
        square_vid_test();  
    } else if (right_test_did_start) {
        
        if (!timer_set) {
            start_test_time = millis();
            timer_set = true;
        }
        right_angle_vid_test();
    }else if (circle_test_did_start) {
        
        if (!timer_set) {
            start_test_time = millis();
            timer_set = true;
        }
        circle_test();
    }else if (spiral_test_did_start) {
        
        if (!timer_set) {
            start_test_time = millis();
            timer_set = true;
        }
        spiral_test();
    }else if (sine_test_did_start) {
      if (!timer_set) {
            start_test_time = millis();
            timer_set = true;
        }
        sinusoid_test();
    } else if (heading_circle_test_did_start) {
      if (!timer_set) {
            start_test_time = millis();
            timer_set = true;
        }
        heading_circle_test();
    } 
}

int increment = 1;
int pwm = 0;
void test_motors() {
    // We have never used this function before.
    // I left it in just in case
    pwm += increment;
    if (pwm > 30) {
        increment = -1;
    } else if (pwm < 1) {
        increment = 1;
        digitalWrite(M1_DIR_PIN, LOW);
        digitalWrite(M2_DIR_PIN, HIGH);
        //    digitalWrite(MT_DIR_PIN, LOW);
    }

    analogWrite(M1_PWM_PIN, pwm);
    analogWrite(M2_PWM_PIN, pwm);
    //  analogWrite(MT_PWM_PIN, pwm);

    Serial.print("M1_PWM_PIN: "); Serial.println(M1_PWM_PIN);
    Serial.print("M2_PWM_PIN: "); Serial.println(M2_PWM_PIN);
    //  Serial.print("MT_PWM_PIN: "); Serial.println(MT_PWM_PIN);
    Serial.print("pwm: "); Serial.println(pwm);
    delay(50);
}

/*SQUARE VIDEO TEST*/
void square_vid_test() {
    if (millis() < start_test_time + 5000) {
        desired_h_xdot = 0.0;
        desired_h_ydot = 0.0;
    }
    else if(millis() < start_test_time + 10000){
        desired_h_xdot = -.2;
        desired_h_ydot = 0;
    } else if(millis() < start_test_time + 15000){
        desired_h_xdot = 0;
        desired_h_ydot = -.2;
    } else if(millis() < start_test_time + 20000){
        desired_h_xdot = .2;
        desired_h_ydot = 0;
    } else if(millis() < start_test_time + 25000){
        desired_h_xdot = 0;
        desired_h_ydot = .2;
    } else {
        square_test_did_start = false;
        timer_set = false;
        desired_h_xdot = 0;
        desired_h_ydot = 0;
    }
}

/*RIGHT ANGLE VIDEO TEST*/
void right_angle_vid_test() {
  
    if(millis() < start_test_time + 3000) {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
    } else if(millis() < start_test_time + 3750){
        desired_h_xdot = 0;
        desired_h_ydot = .75;
    } else if(millis() < start_test_time + 4500){
        desired_h_xdot = -.75;
        desired_h_ydot = 0;
    } else {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        right_test_did_start = false;
        timer_set = false;
    }
}

void circle_test() {
  debugmessage = -1;
  int circle_size = 4;
  if(millis() < start_test_time + 5000) {
    desired_h_xdot = 0;
    desired_h_ydot = 0;
      debugmessage = 1;

  } else if(millis() < start_test_time + 5000 + circle_size*6282) {
  seconds = (float)(millis() - start_test_time-5000)/1000.0;
  R = .2;
    
    desired_h_xdot = (float)-R * (float)sin(seconds/(float)circle_size);  //6.28*circle_size second revolution
  desired_h_ydot = (float)R * (float)cos(seconds/(float)circle_size);
  debugmessage = desired_h_xdot;

  }else {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        circle_test_did_start = false;
        timer_set = false;
          debugmessage = 3;
 
  } 
}

void sinusoid_test() {
  int curve_size = 2;
  if(millis() < start_test_time + 5000) {
    desired_h_xdot = 0;
    desired_h_ydot = 0;
  } else if(millis() < start_test_time + 5000 + 6282*curve_size) {
  seconds = (float)(millis() - start_test_time-5000)/1000.0;
  R = .2;
  
  desired_h_xdot = -R * sin(seconds/(float)curve_size);  //6.28*curve_size second revolution
  desired_h_ydot = R;  
  }else {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        sine_test_did_start = false;
        timer_set = false;
  }
}

void spiral_test() {
  if(millis() < start_test_time + 5000) {
    desired_h_xdot = 0;
    desired_h_ydot = 0;
  } else if(millis() < start_test_time + 35000) {
  seconds = (float)(millis() - start_test_time-5000)/1000.0;
  R = .2*seconds/10;
  
  desired_h_xdot = -R * sin(seconds);  //6.28 second revolution
  desired_h_ydot = R * cos(seconds);  //I think this is going to accelerate?
  } else {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        spiral_test_did_start = false;
        timer_set = false;
  }
}

void heading_circle_test() {
  int circle_size = 4;
  if(millis() < start_test_time + 5000) {
    desired_h_xdot = 0;
    desired_h_ydot = 0;
      debugmessage = 1;
  } else if(millis() < start_test_time + 5000 + circle_size*6282) {
  seconds = (float)(millis() - start_test_time-5000)/1000.0;
  R = .2;
    
  desired_h_xdot = 0;  //6.28*circle_size second revolution
  desired_h_ydot = .2;
  desired_h_rdot = (float)6.282/(((float)circle_size * 6282)/1000);
  debugmessage = desired_h_xdot;

  }else {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        desired_h_rdot = 0;
        heading_circle_test_did_start = false;
        timer_set = false;
          debugmessage = 3;
 
  } 
  }

void zipper_path() {
    if(millis() < start_time + 4000){
      desired_h_xdot = .2;
      desired_h_ydot = 0;
    } else if(millis() < start_time + 8000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else if(millis() < start_time + 12000){
      desired_h_xdot = -.2;
      desired_h_ydot = 0;
    } else if(millis() < start_time + 16000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else if(millis() < start_time + 20000){
      desired_h_xdot = .2;
      desired_h_ydot = 0;
    } else if(millis() < start_time + 24000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else if(millis() < start_time + 28000){
      desired_h_xdot = -.2;
      desired_h_ydot = 0;
    } else if(millis() < start_time + 32000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else {
      desired_h_xdot = 0.0;
      desired_h_ydot = 0.0;
    }
}

void test_ADA() {
    // We've never used this function either
    pwm_M1 = adjust_speed(pwm_M1, desired_M1_v);
    pwm_M2 = adjust_speed(pwm_M2, desired_M2_v);
    pwm_MT = adjust_speed(pwm_MT, desired_MT_v);

    analogWrite(M1_PWM_PIN, pwm_M1);
    analogWrite(M2_PWM_PIN, pwm_M2);
    analogWrite(MT_PWM_PIN, pwm_MT);

    digitalWrite(M1_DIR_PIN, (desired_M1_v >= 0) ? M1_FORWARD : !M1_FORWARD);
    digitalWrite(M2_DIR_PIN, (desired_M2_v >= 0) ? M2_FORWARD : !M2_FORWARD);
    digitalWrite(MT_DIR_PIN, (desired_MT_v >= 0) ? MT_COUNTER : !MT_COUNTER);


    Serial.print("pwm_M1: "); Serial.println(pwm_M1);
    Serial.print("pwm_M2: "); Serial.println(pwm_M2);
    Serial.print("pwm_MT: "); Serial.println(pwm_MT);
    Serial.print("pwm: "); Serial.println(pwm);
    delay(10);
}

int adjust_speed(int pwm, int desired) {
    if (pwm > abs(desired)) {
        pwm = abs(desired);
    } else if (pwm < abs(desired)) {
        pwm++;
    }
    return pwm;
}
