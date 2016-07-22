#include <libas.h>

#include <Wire.h>

#include "pid.h"
#include "motor.h"
#include "localize.h"
#include "hamr_imu.h"
#include "dd_control.h"
#include "constants.h"
#include "holonomic_control.h"

// ROS things 
#include <ros.h>
#include <hamr_test/HamrStatus.h>
#include <hamr_test/MotorStatus.h>
#include <hamr_test/HamrCommand.h>
#include <hamr_test/HoloStatus.h>
#include <hamr_test/VelocityStatus.h>
#include <ros/time.h>
ros::NodeHandle nh;

// Publishing
//hamr_test::HamrStatus hamrStatus;
//hamr_test::MotorStatus leftMotor;
//hamr_test::MotorStatus rightMotor;
//hamr_test::MotorStatus turretMotor;
//ros::Publisher pub("hamr_state", &hamrStatus);

hamr_test::HoloStatus holoStatus;
ros::Publisher pub("holo_state", &holoStatus);

//hamr_test::VelocityStatus velStatus;
//ros::Publisher pub("vel_state", &velStatus);
  
// Subscribing
  // TODO: Make a separate message for commands? 
  // Or we can just continue to use the HamrStatus message and do validations on
  // the arduino side. 
void commandCallback(const hamr_test::HamrCommand& command_msg);
ros::Subscriber<hamr_test::HamrCommand> sub("hamr_command", &commandCallback);



/* -------------------------------------------------------*/
/* These following values are modifiable through serial communication or determined by a control output */

/*this value is updated by the control software. if it is true, the arduino will send data through the send_serial function */
int send_data = 0;

/* DESIRED VALUES */
// holonomic velocities
float desired_h_xdot = 0; float desired_h_ydot = 0; float desired_h_rdot = 0;

// differential drive velocities
float desired_dd_v = 0; // Diff Drive (m/s)
float desired_dd_r = 0; // desired angular velocity for Diff Drive: set between [-1,1] by controller, mapped to [-90,90] degrees in code
// float speed_req_turret = 0.0; // Turret (rad/s)?

// motor velocities
float desired_M1_v = 0; float desired_M2_v = 0; float desired_MT_v = 0;

// motor PWMs
int pwm_M1 = 0;
int pwm_M2 = 0;
int pwm_MT = 0;
float M1_v_cmd = 0;
float M2_v_cmd = 0;
float MT_v_cmd = 0;

/* CONTROL PARAMETERS */
// PID Values are to be changed later- they work enough for now. 
PID_Vars pid_vars_M1(0.6, 10.0, 0.005);
PID_Vars pid_vars_M2(0.6, 10.0, 0.005);
PID_Vars pid_vars_MT(0.005, 0.07, 0.0);
PID_Vars dd_ctrl(0.1, 0.0, 0.0);

// PID_Vars pid_vars_dd_v(1.0, 0.0, 0.0);
// PID_Vars pid_vars_dd_r(1.0, 0.0, 0.0);
PID_Vars pid_vars_h_xdot(0.1, 0.0, 0.0);
PID_Vars pid_vars_h_ydot(0.1, 0.0, 0.0);
PID_Vars pid_vars_h_rdot(0.001, 0.0, 0.0);

// Velocity control command
float h_xdot_cmd = 0; float h_ydot_cmd = 0; float h_rdot_cmd = 0; //holonomic
float dtheta_cmd = 0; //differential drive
/* -------------------------------------------------------*/
/* -------------------------------------------------------*/

/* SENSORS -----------------------------------------------*/
// decoder counts
int decoder_count_M1 = 0;
int decoder_count_M2 = 0;
int decoder_count_MT = 0;

const int AVG_FILT_SZ = 5; // this was originally 5


float decoder_count_arr_M1[AVG_FILT_SZ];
float decoder_count_arr_M2[AVG_FILT_SZ];
float decoder_count_arr_MT[10];

int decoder_count_M1_prev = 0;
int decoder_count_M2_prev = 0;
int decoder_count_MT_prev = 0;

long decoder_turret_total = 0;

// encoders
// Encoder counting interrupt functions
//void rencoderA_M1(); void rencoderB_M1();
//void rencoderA_M2(); void rencoderB_M2();
//void rencoderA_MT(); void rencoderB_MT();

/* encoder counters */

//volatile long curr_count_M1 = 0; volatile long prev_count_M1 = 0;
//volatile long curr_count_M2 = 0; volatile long prev_count_M2 = 0;
volatile long curr_count_MT = 0; volatile long prev_count_MT = 0;
//volatile long curr_count_M1 = 0; volatile long prev_count_M1 = 0;
//volatile long curr_count_M2 = 0; volatile long prev_count_M2 = 0;
//volatile long curr_count_MT = 0; volatile long prev_count_MT = 0;

/* encoder output state */
int interrupt_M1_A = 0; int interrupt_M1_B = 0;
int interrupt_M2_A = 0; int interrupt_M2_B = 0;
int interrupt_MT_A = 0; int interrupt_MT_B = 0;

/* measured velocities */
float sensed_M1_v = 0.0;
float sensed_M2_v = 0.0;
float sensed_MT_v = 0.0;

float sensed_M1_v_prev = 0.0;
float sensed_M2_v_prev = 0.0;
float sensed_MT_v_prev = 0.0;

float sensed_M1_v_filt = 0.0;
float sensed_M2_v_filt = 0.0;
float sensed_MT_v_filt = 0.0;

/* computed holonomic velocities */
float computed_xdot = 0.0;
float computed_ydot = 0.0;
float computed_tdot = 0.0;

/* IMU settings */
const float SENSOR_LOOPTIME = 10; //10000 //remeber to update this to microseconds later, or an exact timing method if possible
unsigned long next_sensor_time = millis(); // micros()
unsigned long prev_micros = 0;
float sensed_drive_angle = 0;

/* -------------------------------------------------------*/
/* -------------------------------------------------------*/

// timing: main loop
unsigned long startMilli;
unsigned long lastMilli = 0;
float t_elapsed;

// dd localization
location hamr_loc;

// Some quick (somewhat hacky) setup
void init_actuators();
void init_I2C();

float pidError; // PID debugging for turret. See pid.h
float dummy1;
float dummy2;

void setup() {
  //  delay(3000);
  nh.initNode(); // Initialize ros node
  nh.subscribe(sub); // arduino node subscribes to topic declared above
  nh.advertise(pub); // advertise the topic that will be published by the arduino
   // Initial value for pidError
  dummy1 = 0.0; // placeholders for PID errors
  dummy2 = 0.0;
  pinMode(40, OUTPUT); // TESTING FOR BLINKING
  
  init_actuators();           // initialiaze all motors
  init_I2C();                 // initialize I2C bus as master

  // delay(2500);
  startMilli = millis(); //startMicro = micros()
}

unsigned long startTestTime;
bool squareTestDidStart = false;
bool rightTestDidStart = false;
bool timerSet = false;

/*SQUARE VIDEO TEST*/
void square_vid_test() {
    if (millis() < startTestTime + 5000) {
      desired_h_xdot = 0.0;
      desired_h_ydot = 0.0;
    }
    else if(millis() < startTestTime + 10000){
      desired_h_xdot = -.2;
      desired_h_ydot = 0;
    } else if(millis() < startTestTime + 15000){
      desired_h_xdot = 0;
      desired_h_ydot = -.2;
    } else if(millis() < startTestTime + 20000){
      desired_h_xdot = .2;
      desired_h_ydot = 0;
    } else if(millis() < startTestTime + 25000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else {
      squareTestDidStart = false;
      timerSet = false;
      desired_h_xdot = 0;
      desired_h_ydot = 0;
    }
}

void forward_test() {
//  Serial.print("going forward...\n");
//  desired_M1_v = 1.0; 
//  desired_M2_v = 1.0;
//  delay(1000);
//  desired_M1_v = 0; 
//  desired_M2_v = 0;
//  Serial.print("Stop.\n");
//  delay(1000);
}

/*RIGHT ANGLE VIDEO TEST*/
void right_angle_vid_test() {
    if(millis() < startTestTime + 6000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else if(millis() < startTestTime + 12000){
      desired_h_xdot = -.2;
      desired_h_ydot = 0;
    } else {
      desired_h_xdot = 0;
      desired_h_ydot = 0;
      rightTestDidStart = false;
      timerSet = false;
    }
}

void zipper_path() {
    if(millis() < startMilli + 4000){
      desired_h_xdot = .2;
      desired_h_ydot = 0;
    } else if(millis() < startMilli + 8000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else if(millis() < startMilli + 12000){
      desired_h_xdot = -.2;
      desired_h_ydot = 0;
    } else if(millis() < startMilli + 16000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else if(millis() < startMilli + 20000){
      desired_h_xdot = .2;
      desired_h_ydot = 0;
    } else if(millis() < startMilli + 24000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    } else if(millis() < startMilli + 28000){
      desired_h_xdot = -.2;
      desired_h_ydot = 0;
    } else if(millis() < startMilli + 32000){
      desired_h_xdot = 0;
      desired_h_ydot = .2;
    }else {
      desired_h_xdot = 0.0;
      desired_h_ydot = 0.0;
    }
}

/***************************/
/* MAIN LOOP
  /***************************/
void compute_sensed_motor_velocities();
void send_serial();

int loop_time_duration;
int offset = 0;
bool didSetOffset = false;

void loop() {
  int i = 0;

  
  while (1) {
    // last timing was between 900 and 1200 microseconds. the range seems high...
    // uncomment the first and last line in while loop to test timing
    // unsigned long start_time = micros();

    //zipper_path();
    //square_vid_test();
    //right_angle_vid_test();

    //forward_test();

    loop_time_duration = millis() - lastMilli;
    
    if ((millis() - lastMilli) >= LOOPTIME) { //micros() - lastMicro()
      //serial communication
      //read_serial();
      

      t_elapsed = (float) (millis() - lastMilli); // (micros() - lastMicros) / 1000.0
      lastMilli = millis();

      //      Serial.println(decoder_count_M1);
      //      unsigned int diff = decoder_count_M1;

      compute_sensed_motor_velocities(); // read encoders

      send_serial();

      if (squareTestDidStart) {
        digitalWrite(40, HIGH);
      } else {
        digitalWrite(40, LOW);
      }

    if (squareTestDidStart) {
      square_vid_test();
      if (!timerSet) {
        startTestTime = millis();
        timerSet = true;
      }  
    } else if (rightTestDidStart) {
      right_angle_vid_test();
      if (!timerSet) {
        startTestTime = millis();
        timerSet = true;
      }
    }

//       DIFFERENTIAL DRIVE CONTROL
//       int use_dd_control = 1;
//       isensed_drive_anglef (use_dd_control == 0) {
//         // PID velocity control, same input to both motors
//         desired_M1_v = -1 * desired_dd_v;
//         desired_M2_v = desired_dd_v;
//       } else if (use_dd_control == 1) {
//         // Differential drive control
//         angle_control(&dd_ctrl, desired_dd_r, hamr_loc.w, &dtheta_cmd, desired_dd_v, &desired_M1_v, &desired_M2_v, WHEEL_DIST, WHEEL_RADIUS, t_elapsed);
//       } else {
//        // use indiv setpoints
//        desired_M1_v = -1 * (desired_dd_v - (WHEEL_DIST/2.0) * PI/2.0);
//        desired_M2_v = (desired_dd_v + (WHEEL_DIST/2.0) * PI/2.0);
//       }

      //M1 is the RIGHT motor and is forward facing caster wheels
      //M2 is LEFT

      /* *********************** */
      /* BEGIN HOLONOMIC CONTROL */
      // compute xdot, ydot, and theta dot using the sensed motor velocties and drive angle


      // AND THIS 
        /*
        STEP 1: CONVERT MOTOR VELOCITY TO ACTUAL TURRET VELOCITY
        
        */
      
      compute_global_state(-1 * sensed_M1_v, sensed_M2_v, sensed_MT_v, 2*PI*sensed_drive_angle,
                           &computed_xdot, &computed_ydot, &computed_tdot);
////      //
        h_xdot_cmd = desired_h_xdot;
        h_ydot_cmd = desired_h_ydot;
        h_rdot_cmd = desired_h_rdot;
      // // 

      // UNCOMMENT THE FOLLOWING LINE TO ENABLE HOLONOMIC PID
      // holonomic PID
//       h_xdot_cmd = pid_vars_h_xdot.update_pid(desired_h_xdot, computed_xdot, t_elapsed);
//       h_ydot_cmd = pid_vars_h_ydot.update_pid(desired_h_ydot, computed_ydot, t_elapsed);
//       h_rdot_cmd = pid_vars_h_rdot.update_pid(desired_h_rdot, computed_tdot * 180 / PI, t_elapsed);

//PUT THIS BACK IN
      // using output of holonomic PID, compute jacobian values for motor inputs
            set_holonomic_desired_velocities(h_xdot_cmd, h_ydot_cmd, h_rdot_cmd); // set these setpoints to the output of the holonomic PID controllers
            get_holonomic_motor_velocities(sensed_drive_angle * 2 * PI, &desired_M1_v, &desired_M2_v, &desired_MT_v);
//            get_holonomic_motor_velocities(hamr_loc.theta, &desired_M1_v, &desired_M2_v, &desired_MT_v);


      set_speed(&pid_vars_M1,
                desired_M1_v,
                sensed_M1_v,
                &M1_v_cmd,
                t_elapsed,
                &pwm_M1,
                M1_DIR_PIN,
                M1_PWM_PIN,
                &dummy1);

       set_speed(&pid_vars_M2,
                 desired_M2_v,
                 sensed_M2_v,
                 &M2_v_cmd,
                 t_elapsed,
                 &pwm_M2,
                 M2_DIR_PIN,
                 M2_PWM_PIN,
                 &dummy2);
//                 
//      set_speed(&pid_vars_MT,
//                desired_MT_v,
//                sensed_MT_v,
//                &MT_v_cmd,
//                t_elapsed,
//                &pwm_MT,
//                MT_DIR_PIN,
//                MT_PWM_PIN,
//                &pidError);
//
      set_speed_of_turret(&pid_vars_MT,
                desired_MT_v,
                sensed_MT_v,
                &MT_v_cmd,
                t_elapsed,
                &pwm_MT,
                MT_DIR_PIN,
                MT_PWM_PIN,
                &pidError);
        
      
      //      Serial.println(decoder_count_M1);
      //      diff = diff - decoder_count_M1;
      //      Serial.println(diff);




            
      //
      // Serial.print(hamr_loc.w);  Serial.print(" ");
      // Serial.println(computed_tdot);
      // Serial.println(computed_tdot - sensed_MT_v);

      // Serial.print(desired_MT_v); Serial.print(" ");
      // desired_MT_v = -(hamr_loc.w + h_rdot_cmd);
      // Serial.println(desired_MT_v);

      // Serial.println(desired_MT_v);

      // Serial.println(desired_M1_v);
      
      //desired_MT_v *= 180.0 / PI;

      // desired_MT_v is

      // Serial.println(desired_MT_v);
      /* END HOLONOMIC CONDTROL */
      /* ********************** */

      // Serial.println(desired_MT_v);
      
//
//      Serial.print("Desired\n");
//      Serial.print(desired_M1_v);
//      Serial.print("\n");
//      Serial.print("PWM\n");
//      Serial.print(M1_PWM_PIN);
//      Serial.print("\n");
    }

    //analogWrite(MT_PWM_PIN, 50);

    // Serial.print(desired_MT_v); Serial.print(" sensed ");
    // Serial.println(sensed_MT_v); Serial.println(decoder_count_MT);


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
    
    //sensed_drive_angle = 1 - sensed_drive_angle;


    
    // Serial.println(sensed_drive_angle);

    // unsigned long finish_time = micros();
    // Serial.print("total_time: "); Serial.println(finish_time - start_time);
    // Test Loop


    //square_vid_test();
  }
}

/***************************/
/* END MAIN LOOP
  /***************************/

/******************************************/
/* BEGIN SERIAL COMMUNCIATION CODE
  /******************************************/

/* read a byte from Serial. Perform appropriate action based on byte*/

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
      case SIG_START_LOG:
        send_data = 1;
        break;

      case SIG_STOP_LOG:
        send_data = 0;
        break;

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
        squareTestDidStart = true;
        break;
      case -101:
        rightTestDidStart = true;
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

    holoStatus.setpoint_x =  (int)(h_xdot_cmd * 1000);
    holoStatus.setpoint_y = (int)(h_ydot_cmd * 1000);
    holoStatus.setpoint_r = (int)(h_rdot_cmd * 1000);
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
//    velStatus.sensed_t_motor_velocity = (int) (((float(turret_tick_change)/1023))/(t_elapsed/1000) * 1000);
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
//  sensed_M1_v = get_speed(decoder_count_change_filt_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
//  sensed_M2_v = get_speed(decoder_count_change_filt_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
//  sensed_MT_v = get_speed(decoder_count_change_filt_MT, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
//  sensed_MT_v = get_ang_speed(decoder_count_change_filt_MT, TICKS_PER_REV_TURRET, t_elapsed);

// Low-pass Filter
  float currentVelRight = get_speed_from_difference(decoder_count_change_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
  float currentVelLeft = get_speed_from_difference(decoder_count_change_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, t_elapsed);
  float currentVelTurret = get_ang_speed_from_difference(decoder_count_change_MT, TICKS_PER_REV_TURRET, t_elapsed);
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

  hamr_loc.update(sensed_M1_v, sensed_M2_v, WHEEL_DIST, t_elapsed);
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
