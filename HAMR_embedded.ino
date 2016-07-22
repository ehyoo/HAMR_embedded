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
#include <Wire.h> // I2C Communication, 
#include <libas.h> // SSI Communication

/*******************************/
/*   ROS Serial Communication  */
/*******************************/
// Message Importing
#include <ros.h>
#include <hamr_test/HamrStatus.h>
#include <hamr_test/MotorStatus.h>
#include <hamr_test/HamrCommand.h>
#include <hamr_test/HoloStatus.h>
#include <hamr_test/VelocityStatus.h>
#include <ros/time.h>

// Node and Publishing
ros::NodeHandle nh;
hamr_test::HamrStatus hamrStatus;
hamr_test::MotorStatus leftMotor;
hamr_test::MotorStatus rightMotor;
hamr_test::MotorStatus turretMotor;
// By default, should publish to hamr_state topic
ros::Publisher pub("hamr_state", &hamrStatus);

// Holonomic debugging 
// hamr_test::HoloStatus holoStatus;
// ros::Publisher pub("holo_state", &holoStatus);

// Turret Velocity debugging messages
// hamr_test::VelocityStatus velStatus;
// ros::Publisher pub("vel_state", &velStatus);
  
// Subscribing
void commandCallback(const hamr_test::HamrCommand& command_msg);
ros::Subscriber<hamr_test::HamrCommand> sub("hamr_command", &commandCallback);

/***************************************************/
/*                                                 */
/*                   VARIABLES                     */
/*                                                 */
/***************************************************/

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
// TODO: This will eventually overflow after a long time 
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
PID_Vars pid_vars_M1(0.6, 10.0, 0.005);
PID_Vars pid_vars_M2(0.6, 10.0, 0.005);
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
bool square_test_did_start = false;
bool right_test_did_start = false;
bool timer_set = false;

/***************************/
/*          Modes          */
/***************************/
// Use direct drive by sending false to both dif_drive and holonomic_drive
bool use_dif_drive = true; // Sending linear velocity commands and turret commands
bool use_holonomic_drive = false; // Full fledged holonomic drive

/***********************/
/*    Miscellaneous    */
/***********************/
// Initial angle offset 
int offset = 0;
bool didSetOffset = false;

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
        if ((millis() - last_recorded_time) >= LOOPTIME) { // stable loop time
            time_elapsed = (float) (millis() - last_recorded_time);
            last_recorded_time = millis();
            compute_sensed_motor_velocities(); // read encoders
            send_serial(); // Send the current state via ROS
            check_for_test_execution(); // takes care of drive demo test commands
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

        float ticks = TICKS_PER_REV_TURRET;
        sensed_drive_angle = fmod(decoder_turret_total, ticks) / (float) ticks;
        if (sensed_drive_angle < 0) {
            sensed_drive_angle = 1 + sensed_drive_angle;
        } 
        if (!didSetOffset) {
            offset = sensed_drive_angle;
            didSetOffset = true;
        }
        sensed_drive_angle = sensed_drive_angle - offset;
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
    compute_global_state(-1 * sensed_M1_v, 
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

/******************************************/
/* BEGIN SERIAL COMMUNCIATION CODE
  /******************************************/

// NOTE: this is going to be the callback
void commandCallback(const hamr_test::HamrCommand& command_msg) {
  // the HamrCommand msg is detailed as follows: 
  // string type (the type that corresponds to the switch cases)
  // string val (the value of the float)
  String str;
  float temp;
  float* sig_var;
  char type = static_cast<char>(command_msg.type);
  String val = command_msg.val;

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
//        sig_var = &(pid_vars_MT.Kp);
         sig_var = &(dd_ctrl.Kp);
        break;

      case SIG_T_KI:
//        sig_var = &(pid_vars_MT.Ki);
         sig_var = &(dd_ctrl.Ki);
        break;

      case SIG_T_KD:
//        sig_var = &(pid_vars_MT.Kd);
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
        square_test_did_start = true;
        break;
      case -101:
        right_test_did_start = true;
        break;
    }
      *sig_var = val.toFloat();
}

/* send relevant data through serial
  Everything in the if statement takes between 1200 and 1300 microseconds
  uncomment first and last line to test timing */

  int turret_tick_change; // For debugging purposes for sending through serial- you should delete this later
  
void send_serial() {
//    leftMotor.position = decoder_count_M2;
//    rightMotor.position = decoder_count_M1;
//    turretMotor.position = decoder_count_MT;
//    // Arbitrary 1000 multiplied to ensure that we can send it over as an int. 
//    // Just think of it as mm/s 
//    // This should be fixed soon.
//    leftMotor.velocity = (int)(sensed_M2_v * 1000);
//    rightMotor.velocity = (int)(sensed_M1_v * 1000);
//    turretMotor.velocity = (int)(sensed_MT_v * 100);
//    leftMotor.desired_velocity = (int)(desired_M2_v * 1000);
//    rightMotor.desired_velocity = (int)(desired_M1_v * 1000);
//    turretMotor.desired_velocity = (int)(desired_MT_v * 100);
//    // These should be deleted later- these were put into messages purely for debugging
//    turretMotor.speed_cmd = (int)(roundf(MT_v_cmd * 100));
//    leftMotor.speed_cmd = 0;
//    rightMotor.speed_cmd = 0;
//    turretMotor.pidError = (int)(roundf(pidError * 100));
//    leftMotor.pidError = 0;
//    rightMotor.pidError = 0;
//    hamrStatus.timestamp = nh.now();
//    hamrStatus.left_motor = leftMotor;
//    hamrStatus.right_motor = rightMotor;
//    hamrStatus.turret_motor = turretMotor;
//    hamrStatus.looptime = loop_time_duration;
//    pub.publish(&hamrStatus);

//    holoStatus.setpoint_x =  (int)(h_xdot_cmd * 1000);
//    holoStatus.setpoint_y = (int)(h_ydot_cmd * 1000);
//    holoStatus.setpoint_r = (int)(h_rdot_cmd * 1000);
//    holoStatus.xdot = (int)(computed_xdot*1000);
//    holoStatus.ydot = (int)(computed_ydot*1000);
//    holoStatus.tdot = (int)(computed_tdot*100);
//    holoStatus.left_vel = (int)(sensed_M2_v * 1000);
//    holoStatus.right_vel = (int)(sensed_M1_v * 1000);
//    holoStatus.turret_vel = (int)(sensed_MT_v * 100);
//    holoStatus.desired_left_vel = (int) (desired_M2_v * 1000);
//    holoStatus.desired_right_vel = (int) (desired_M1_v * 1000);
//    holoStatus.desired_turret_vel = (int) (desired_MT_v * 100);
//    holoStatus.sensed_drive_angle = (int)(sensed_drive_angle*360);
//    pub.publish(&holoStatus);


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

/******************************************/
/* END SERIAL COMMUNICATION CODE
  /******************************************/


/******************************************/
/* BEGIN MOTOR CODE
  /******************************************/
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

float compute_avg(float* arr, int sz) {
  float sum = 0;
  for (int i = 0; i < sz; i++) {
    sum += arr[i];
  }
  return sum / (float) sz;
}

float prevSensedVelocityRight;
float prevSensedVelocityLeft;
float prevSensedVelocityTurret;

void compute_sensed_motor_velocities() {
  // read decoders to get encoder counts

  // [ED]- CHANGE HERE 
  // uses the libas library to read from pins
  // pins are CLK, DI, CSn- currently arbitrarily set to 1. 
  // TODO: CHANGE THESE PINS LATER


  // DUDE CHANGE THIS.
  libas * libas_M1 = new libas(30, 32, 28, 12); 
  libas * libas_M2 = new libas(24, 26, 22, 12); 
  libas * libas_MT = new libas(38, 34, 36, 10);

  decoder_count_M1 = libas_M1->GetPosition();
  decoder_count_M2 = libas_M2->GetPosition();
  decoder_count_MT = libas_MT->GetPosition();
  delete libas_M1;
  delete libas_M2;
  delete libas_MT;
  // =========

  // in sudden jumps of encoder values: 
  float decoder_count_change_M1;
  float decoder_count_change_M2;
  float decoder_count_change_MT;

  // We could (and should) make this DRYer
  if (decoder_count_M1_prev > 3500 && decoder_count_M1 < 500) {
    decoder_count_change_M1 = 4095 - decoder_count_M1_prev + decoder_count_M1;
  } else if (decoder_count_M1_prev < 500 && decoder_count_M1 > 3500) {
    decoder_count_change_M1 = -1 * (4095 - decoder_count_M1 + decoder_count_M1_prev);
  } else {
    decoder_count_change_M1 = decoder_count_M1 - decoder_count_M1_prev;
  }

  if (decoder_count_M2_prev > 3500 && decoder_count_M2 < 500) {
    decoder_count_change_M2 = 4095 - decoder_count_M2_prev + decoder_count_M2;
  } else if (decoder_count_M2_prev < 500 && decoder_count_M2 > 3500) {
    decoder_count_change_M2 = -1 * (4095 - decoder_count_M2 + decoder_count_M2_prev);
  } else {
    decoder_count_change_M2 = decoder_count_M2 - decoder_count_M2_prev;
  }

  /* 
  aggregateCount


  */
//  if (decoder_count_MT_prev > 3500 && decoder_count_MT < 500) {
//    decoder_count_change_MT = 4095 - decoder_count_MT_prev + decoder_count_MT;
//  } else if (decoder_count_MT_prev < 500 && decoder_count_MT > 3500) {
//    decoder_count_change_MT = -1 * (4095 - decoder_count_MT + decoder_count_MT_prev);
//  } else {
//    decoder_count_change_MT = decoder_count_MT - decoder_count_MT_prev;
//  }

    //decoder_count_MT = 1023 - decoder_count_MT;
    if (decoder_count_MT_prev > 700 && decoder_count_MT < 300) {
      decoder_count_change_MT = 1023 - decoder_count_MT_prev + decoder_count_MT;
    } else if (decoder_count_MT_prev < 300 && decoder_count_MT > 700) {
      decoder_count_change_MT = -1 * (1023 - decoder_count_MT + decoder_count_MT_prev);
    } else {
      decoder_count_change_MT = decoder_count_MT - decoder_count_MT_prev;
    }
   turret_tick_change = decoder_count_change_MT;


  decoder_turret_total -= decoder_count_change_MT;

  decoder_count_M1_prev = decoder_count_M1;
  decoder_count_M2_prev = decoder_count_M2;
  decoder_count_MT_prev = decoder_count_MT;

  // [ed]- commented out
//  Serial.print(decoder_count_M1);
//  Serial.print(" ");
//  Serial.print(decoder_count_M2);
//  Serial.print(" ");
//  Serial.print(decoder_count_MT);
//  Serial.print("\n");

  // Moving average filter on decoder count differences
//  for (int i = 1; i < AVG_FILT_SZ; i++) {
//    decoder_count_arr_M1[i] = decoder_count_arr_M1[i - 1];
//    decoder_count_arr_M2[i] = decoder_count_arr_M2[i - 1];
//    
//  }
//  for (int j = 1; j < 10; j++) {
//    decoder_count_arr_MT[j] = decoder_count_arr_MT[j - 1];
//  }
//  decoder_count_arr_M1[0] = decoder_count_change_M1;
//  decoder_count_arr_M2[0] = decoder_count_change_M2;
//  decoder_count_arr_MT[0] = decoder_count_change_MT;
//  float decoder_count_change_filt_M1 = compute_avg(decoder_count_arr_M1, AVG_FILT_SZ);
//  float decoder_count_change_filt_M2 = compute_avg(decoder_count_arr_M2, AVG_FILT_SZ);
//  float decoder_count_change_filt_MT = compute_avg(decoder_count_arr_MT, AVG_FILT_SZ);

  // compute robot velocities
//  sensed_M1_v = get_speed(decoder_count_change_filt_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed);
//  sensed_M2_v = get_speed(decoder_count_change_filt_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed);
//  sensed_MT_v = get_speed(decoder_count_change_filt_MT, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed);
//  sensed_MT_v = get_ang_speed(decoder_count_change_filt_MT, TICKS_PER_REV_TURRET, time_elapsed);

// Low-pass Filter
  float currentVelRight = get_speed_from_difference(decoder_count_change_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed
  );
  float currentVelLeft = get_speed_from_difference(decoder_count_change_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed
  );
  float currentVelTurret = get_ang_speed_from_difference(decoder_count_change_MT, TICKS_PER_REV_TURRET, time_elapsed
  );
//
  float beta = 0.6; //0.386;
  
  sensed_M1_v = beta * currentVelRight + (1 - beta) * prevSensedVelocityRight;
  sensed_M2_v = beta * currentVelLeft + (1 - beta) * prevSensedVelocityLeft;
  sensed_MT_v = beta * currentVelTurret + (1 - beta) * prevSensedVelocityTurret;

  // M1 and M2 returning m/s
  // sensed_MT_v returning degrees/s

  prevSensedVelocityRight = currentVelRight;
  prevSensedVelocityLeft = currentVelLeft;
  prevSensedVelocityTurret = currentVelTurret;

  // a = beta * (encoderB) + (1 - beta) * encoderBOld (beta is <= 1)

  hamr_loc.update(sensed_M1_v, sensed_M2_v, WHEEL_DIST, time_elapsed
  );
}

/* initialize encoder interrupts for turret motor */
// void init_encoder_interrupts(){
//   pinMode(PIN_MT_ENCODER_A, INPUT);
//   // pinMode(PIN_MT_ENCODER_B, INPUT);

//   attachInterrupt(PIN_MT_ENCODER_A, rencoderA_MT, CHANGE);
//   attachInterrupt(PIN_MT_ENCODER_B, rencoderB_MT, CHANGE);
// }

// encoder interrupts handlers
// void rencoderA_M1()  {
//   interrupt_M1_A = (PIOB->PIO_PDSR >> 17) & 1;
//   if (interrupt_M1_A != interrupt_M1_B) curr_count_M1--; // encoderA changed before encoderB -> forward
//   else                                  curr_count_M1++; // encoderB changed before encoderA -> reverse
// }

// void rencoderB_M1()  {
//   interrupt_M1_B = (PIOB->PIO_PDSR >> 18) & 1;
//   if (interrupt_M1_A != interrupt_M1_B) curr_count_M1++; // encoderB changed before encoderA -> reverse
//   else                                  curr_count_M1--; // encoderA changed before encoderB -> forward
// }

// void rencoderA_M2()  {
//   interrupt_M2_A = (PIOB->PIO_PDSR >> 19) & 1;
//   if (interrupt_M2_A != interrupt_M2_B) curr_count_M2--; // encoderA changed before encoderB -> forward
//   else                                  curr_count_M2++; // encoderB changed before encoderA -> reverse
// }

// void rencoderB_M2()  {
//   interrupt_M2_B = (PIOB->PIO_PDSR >> 20) & 1;
//   if (interrupt_M2_A != interrupt_M2_B) curr_count_M2++; // encoderB changed before encoderA -> reverse
//   else                                  curr_count_M2--; // encoderA changed before encoderB -> forward
// }

// void rencoderA_MT()  {
//   interrupt_MT_A = (PIOB->PIO_PDSR >> 21) & 1;
//   if (interrupt_MT_A != interrupt_MT_B) curr_count_MT--; // encoderA changed before encoderB -> forward
//   else                                  curr_count_MT++; // encoderB changed before encoderA -> reverse
// }

// void rencoderB_MT()  {
//   interrupt_MT_B = (PIOB->PIO_PDSR >> 22) & 1;
//   if (interrupt_MT_A != interrupt_MT_B) curr_count_MT++; // encoderB changed before encoderA -> reverse
//   else                                  curr_count_MT--; // encoderA changed before encoderB -> forward
// }


/*
  Testing functions
*/
int increment = 1;
int pwm = 0;
void test_motors() {

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

/******************************************/
/* END MOTOR CODE
  // /******************************************/


/******************************************/
/* BEGIN I2C CODE
  /******************************************/
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

/******************************************/
/* END I2C CODE
  /******************************************/


/* TEST FUNCTION. INSERT ALL TESTING IN HERE. make sure to comment these out while not testing, otherwise infinite loop */
void read_serial();

void test() {
  delayMicroseconds(1000000);

  while (0) {
    test_ADA();
    read_serial();
  }
  while (1) {
    test_motors();
  }

  //  while(1){
  //    test_I2C_decoder_count();
  //  }
}


void test_ADA() {
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

/***************************************************/
/*                                                 */
/*              Holonomic Drive Tests              */
/*                                                 */
/***************************************************/
void check_for_test_execution() {
  // checks if the HAMR should start to execute any tests
    if (square_test_did_start) {
        square_vid_test();
        if (!timer_set) {
            start_test_time = millis();
            timer_set = true;
        }  
    } else if (right_test_did_start) {
        right_angle_vid_test();
        if (!timer_set) {
            start_test_time = millis();
            timer_set = true;
        }
    }
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
    if(millis() < start_test_time + 6000){
        desired_h_xdot = 0;
        desired_h_ydot = .2;
    } else if(millis() < start_test_time + 12000){
        desired_h_xdot = -.2;
        desired_h_ydot = 0;
    } else {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        right_test_did_start = false;
        timer_set = false;
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
    }else {
      desired_h_xdot = 0.0;
      desired_h_ydot = 0.0;
    }
}
