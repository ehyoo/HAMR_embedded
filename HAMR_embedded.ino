/*************************************/
/*                                   */
/*          HAMR_embedded.ino        */
/*               MODLAB              */
/*                                   */
/*************************************/
// Notes:
// M1 => RIGHT MOTOR
// M2 => LEFT MOTOR
// MT => TURRET MOTOR
#include <Wire.h> // I2C 
#include <AS5048A.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "pid.h"
#include "motor.h"
#include "localize.h"
#include "hamr_imu.h"
#include "dd_control.h"
#include "constants.h"
#include "holonomic_control.h"
#include "message_types.h"
#include "message_manager.h"



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

/************************/
/*    Computed Values   */
/************************/
// Computed holonomic velocities
float computed_xdot = 0.0;
float computed_ydot = 0.0;
float computed_tdot = 0.0;

// Summation of turret 
long decoder_turret_total = 0; // Be careful of overflow

/******************/
/* Set Speed Vars */
/******************/
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
int decoder_count_M1 = 0;
int decoder_count_M2 = 0;
int decoder_count_MT = 0;

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
float time_elapsed; // microseconds
float time_elapsed_millis; // time_elapsed converted 
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
AS5048A angle_sensor_M1(11);
AS5048A angle_sensor_M2(7);
AS5048A angle_sensor_MT(6);

/***********************/
/*    Miscellaneous    */
/***********************/
// Initial angle offset 
float offset = 0;
bool did_set_offset = false;

// dd localization
location hamr_loc; // This isn't being used right now. 
float pidError; // PID debugging for turret. See pid.h
float dummy1 = 0; // serves as dummy placeholder from original code.
float dummy2 = 0;

/***********************/
/*         Wifi        */
/***********************/
int status = WL_IDLE_STATUS;
char ssid[] = "hamr_net"; // network name
char pass[] = "1231231234"; // WEP password. Change when appropraite
int keyIndex = 0;
WiFiServer server(80);
unsigned int local_port = 2390; // arbitrary local port selected to listen on
char packet_buffer[255]; // buffer to hold incoming packet
WiFiUDP Udp;
MESSAGE_MANAGER_t *msg_manager =(MESSAGE_MANAGER_t*)malloc(sizeof(MESSAGE_MANAGER_t)); // create message_manager

/***************************************************/
/*                                                 */
/*                      SETUP                      */
/*                                                 */
/***************************************************/

/**
 * Sets up the serial, motors, AP, and encoders
 */
void setup() {
    Serial.println("Setting up serial...");    
    Serial.begin(57600);
    Serial.println("Done setting serial");
    Serial.println("Setting up motors...");
    init_actuators();           // initialiaze all motors
    Serial.println("Done setting motors");
    Serial.println("Setting up Wifi...");
    start_wifi();
    Serial.println("Done setting Wifi");
    //init_I2C();               // initialize I2C bus as master
    start_time = millis();      // Start timer
    Serial.println("Initializing the sensors");
    init_angle_sensors();
    Serial.println("Done initializing the sensors.");
    Serial.println("HAMR Ready!");
}

/**
 * Initializes the angle sensors and sets them up for reading.
 */
void init_angle_sensors() {
    angle_sensor_M1.init();
    angle_sensor_M1.close();// close after each init to allow spi to start again
    angle_sensor_M2.init();// i made the clock 10Mhz. it was 1Mhz to star
    angle_sensor_M2.close();// close after each init to allow spi to start again
    angle_sensor_MT.init();// i made the clock 10Mhz. it was 1Mhz to star
}

void start_wifi() {
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi Shield is not detected- check if correct Arduino is being used and/or WiFi shield connections");
    while (true);
  }
  Serial.println("Creating access point named:");
  Serial.println(ssid);
  status = WiFi.beginAP(ssid, 1, pass, 1);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating the access point failed.");
    while (true);
  }
  delay(5000);
  server.begin();
  print_wifi_status();
  Udp.begin(local_port);
}

/**
 * Prints the Arduino AP's name, IP, and signal strength
 */
void print_wifi_status() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

/**
 * Checks whether some device connects to the Arduino AP.
 */
void check_wifi_status() {
  if (status != WiFi.status()) {
    status = WiFi.status();
    Serial.println("WiFi status changed: ");
    if (status == WL_AP_CONNECTED) {
      Serial.println("A device has connected to the HAMR Access Point");
    } else {
      Serial.println("A device has disconnected from the HAMR Access point: ");
    }
  }
}

/**
 * Debugging convenience method that blinks the builtin LED.
 */
bool toggle = HIGH;
void blink_times(int i) {
  for (int j = 0; j < i*2; j++) {
    digitalWrite(LED_BUILTIN, toggle);
    delay(500);
    toggle = !toggle;
  }
}

/***************************************************/
/*                                                 */
/*                      MAIN                       */
/*                                                 */
/***************************************************/
void loop() {
    loop_time_duration = micros() - last_recorded_time;
    if ((micros() - last_recorded_time) >= LOOPTIME) { // ensures stable loop time
        time_elapsed = (float) (micros() - last_recorded_time);
        time_elapsed_millis = converted_time_elapsed();
        last_recorded_time = micros();
        check_wifi_status();
        check_incoming_messages();
        compute_sensed_motor_velocities();
        calculate_sensed_drive_angle();
        // check_for_test_execution(); // takes care of drive demo test commands.TODO prevent this from running test if kill command was sent
        if (use_holonomic_drive) {
            holonomic_drive();
        } 
        set_speed_of_motors();
    }
}

/* Debugging Methods- Insert into the main loop as necessary. */
/**
 * Debugging method that prints the motor velocities calculated from the encoder readings.
 */
void print_actual_motor_velocities() {
    Serial.println("Sensed M1 velocity: " + String(sensed_M1_v));
    Serial.println("Sensed M2 velocity: " + String(sensed_M2_v));
    Serial.println("Sensed MT velocity: " + String(sensed_MT_v));
}

/**
 * Debugging method that prints the desired motor velocities set from the client.
 */
void print_desired_motor_velocities() {
    Serial.println("Desired M1 Velocity: " + String(M1_v_cmd));
    Serial.println("Desired M2 Velocity: " + String(M2_v_cmd));
    Serial.println("Desired MT Velocity: " + String(MT_v_cmd)); 
}

/**
 * Debugging method that prints the desired holonomic velocities set from the client.
 */
void print_desired_holonomic_velocities() {
    Serial.println("Desired X: " + String(desired_h_xdot));
    Serial.println("Desired Y: " + String(desired_h_ydot));
    Serial.println("Desired T: " + String(desired_h_rdot));
}

/**
 * Debugging method that prints the PID errors accrued.
 */
void print_pid_errors() {
    Serial.println("M1: " + String(pid_vars_M1.error_acc));
    Serial.println("M2: " + String(pid_vars_M2.error_acc));
    Serial.println("MT: " + String(pid_vars_MT.error_acc));
}

/**
 * Returns the microsecond measured time converted to milliseconds
 * @returns Milliseconds of time_elapsed
 */
 float converted_time_elapsed() {
    return time_elapsed * 0.001;
 }

/************************/
/*   Driving Functions  */
/************************/
/**
 * Activates holonomic drive- calculates the desired motor velocities from the desired holonomic velocities, then adjusts the motors.
 */
void holonomic_drive() {
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
    // using output of holonomic PID, compute jacobian values for motor inputs
    set_holonomic_desired_velocities(h_xdot_cmd, h_ydot_cmd, h_rdot_cmd); // set these setpoints to the output of the holonomic PID controllers
    get_holonomic_motor_velocities(sensed_drive_angle * 2 * PI, &desired_M1_v, &desired_M2_v, &desired_MT_v);
}

/**
 * Gets the sensed drive angle of the current turret with respect to the initial starting state of the turret.
 * The initial state of the turret is set to an arbitrary 0- so there is no "universal 0".
 */
void calculate_sensed_drive_angle() {
    float ticks = TICKS_PER_REV_TURRET;
    sensed_drive_angle = fmod(decoder_turret_total, ticks) / (float) ticks;
    float orig_ang = sensed_drive_angle;
    if (sensed_drive_angle < 0) {
        sensed_drive_angle = 1 + sensed_drive_angle;
    } 
    if (!did_set_offset) {
        offset = sensed_drive_angle;
        did_set_offset = true;
    }
    sensed_drive_angle = sensed_drive_angle - offset;
}

/**
 * Sets the velocities of the motors from the desired motor velocity values.
 */
void set_speed_of_motors() {
    // sets the speed of all three of the motors
    set_speed(&pid_vars_M1,
            desired_M1_v,
            sensed_M1_v,
            &M1_v_cmd,
            time_elapsed_millis,
            &pwm_M1,
            M1_DIR_PIN,
            M1_PWM_PIN,
            &dummy1);
    set_speed(&pid_vars_M2,
            desired_M2_v,
            sensed_M2_v,
            &M2_v_cmd,
            time_elapsed_millis,
            &pwm_M2,
            M2_DIR_PIN,
            M2_PWM_PIN,
            &dummy2);
    set_speed_of_turret(&pid_vars_MT,
                        desired_MT_v,
                        sensed_MT_v,
                        &MT_v_cmd,
                        time_elapsed_millis,
                        &pwm_MT,
                        MT_DIR_PIN,
                        MT_PWM_PIN,
                        &pidError);
}

/**********************/
/* Messaging Protocol */
/**********************/
/**
 * Checks if there are any messages on the message queue and handles message if necessary
 */
void check_incoming_messages() {
    int packet_size = Udp.parsePacket();
    if (packet_size) {
      int len = Udp.read(packet_buffer, 255);
      if (len > 0) {
          packet_buffer[len] = 0;
      }
      handle_message(msg_manager, packet_buffer);
    }    
}

/**
 * Message handler that does necessary actions with the information provided.
 * @param msg_manager Instance of the message manager to identify incoming messages
 * @param val The message packet that has yet to be identified.
 */
void handle_message(MESSAGE_MANAGER_t* msg_manager, char* val) {
    uint8 msg_id = val[0];
    switch (msg_id) {
        case HolonomicVelocityMessageType: {
            use_holonomic_drive = true;
            HolonomicVelocity *msg = (HolonomicVelocity*) val;
            msg_manager->holo_vel_struct = *msg;
            desired_h_xdot = msg_manager->holo_vel_struct.x_dot;
            desired_h_ydot = msg_manager->holo_vel_struct.y_dot;
            desired_h_rdot = msg_manager->holo_vel_struct.r_dot;
            break;
        }

        case DifDriveVelocityMessageType: {
          DifDriveVelocity *msg = (DifDriveVelocity*) val;
          msg_manager->dif_drive_vel_struct = *msg;
          use_holonomic_drive = false;
          desired_M1_v = msg_manager->dif_drive_vel_struct.left_v;
          desired_M2_v = msg_manager->dif_drive_vel_struct.right_v;
          desired_MT_v = msg_manager->dif_drive_vel_struct.turret_v;
          break;
        }
        
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
            
        default:
            Serial.println("Message not recognized.");
            break;
    }
}




/******************************************************/
/*      Encoder reading and Velocity Calculations     */
/******************************************************/
/**
 * Initializing for drive.
 */
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

/**
 * Computes the instantaneous velocity from the previous and current encoder reading, then passes it through a filter.
 */
void compute_sensed_motor_velocities() {
    // Decoder count is offset from the maximum encoder value- fixes different encoder orientation from previous encoders.
    decoder_count_M1 = TICKS_PER_REV_DDRIVE - (int) angle_sensor_M1.getRawRotation();
    decoder_count_M2 = TICKS_PER_REV_DDRIVE - (int) angle_sensor_M2.getRawRotation();
    decoder_count_MT = TICKS_PER_REV_DDRIVE - (int) angle_sensor_MT.getRawRotation();
    // Clear the angle sensor errors.
    angle_sensor_M1.getErrors();
    angle_sensor_M2.getErrors();
    angle_sensor_MT.getErrors();
    // Calculating the difference between prev and current sensed positions
    float decoder_count_change_M1 = calculate_decoder_count_change(decoder_count_M1_prev, decoder_count_M1, 16383, 1600, 14000);
    float decoder_count_change_M2 = calculate_decoder_count_change(decoder_count_M2_prev, decoder_count_M2, 16383, 1600, 14000);
    float decoder_count_change_MT = calculate_decoder_count_change(decoder_count_MT_prev, decoder_count_MT, 16383, 1600, 14000);
    // Puts change into the total so we know the drive angle of turret
    decoder_turret_total -= decoder_count_change_MT;
    // Setting the previous encoder values
    decoder_count_M1_prev = decoder_count_M1;
    decoder_count_M2_prev = decoder_count_M2;
    decoder_count_MT_prev = decoder_count_MT;
    // Get current velocities from differences
    float current_vel_right = get_speed_from_difference(decoder_count_change_M1, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed_millis); // m/s
    float current_vel_left = get_speed_from_difference(decoder_count_change_M2, TICKS_PER_REV_DDRIVE, DIST_PER_REV, time_elapsed_millis); // m/s
    float current_vel_turret = get_ang_speed_from_difference(decoder_count_change_MT, TICKS_PER_REV_TURRET, time_elapsed_millis); // degrees/s
    // Pass instantaneous velocities into low pass filter and set them as the sensed velocities
    sensed_M1_v = low_pass_velocity_filter(current_vel_right, prev_sensed_velocity_right);
    sensed_M2_v = low_pass_velocity_filter(current_vel_left, prev_sensed_velocity_left);
    sensed_MT_v = low_pass_velocity_filter(current_vel_turret, prev_sensed_velocity_turret);
    // then assign current velocity to previously sensed velocity
    prev_sensed_velocity_right = current_vel_right;
    prev_sensed_velocity_left = current_vel_left;
    prev_sensed_velocity_turret = current_vel_turret;
}

/**
 * Simple low pass filter.
 * 
 * @param current Current velocity read
 * @param prev Previous velocity read
 */
float low_pass_velocity_filter(float current, float prev) {
    // Simple low pass filter
    float beta = 0.386; // Calculated 0.386, but 0.6 works well
    return beta * current + (1 - beta) * prev; //filter at 10 hz - Tarik.
}

/**
 * Calculates the difference of ticks between previously sensed and currently sensed readings. Also considers the 
 * case when there is an overflow between readings (ie, encoder reads 4090, moves forward, then reads 2 at time t+1)
 * 
 * @param prev Previously sensed value
 * @param current Currently sensed value
 * @param max The encoder's max reading
 * @param lim_min Desired floor at which you determine there has been an overflow
 * @param lim_max Desired ceiling at which you determine that there has been an overflow.
 * @returns The absolute change between the prev and current values.
 */
float calculate_decoder_count_change(int prev, int current, int max, int lim_min, int lim_max) {
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
// NOTE: Feb 3, 2017- Drive Tests have not been tested with the new code base. 
// There is no messaging protocol that triggers the tests- they should be made.
/**
 * Checks if the flag has been set to begin a behavior test.
 */
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
    } else if (millis() < start_test_time + 5000 + circle_size*6282) {
        seconds = (float)(millis() - start_test_time-5000)/1000.0;
        R = .2;
        desired_h_xdot = (float)-R * (float)sin(seconds/(float)circle_size);  //6.28*circle_size second revolution
        desired_h_ydot = (float)R * (float)cos(seconds/(float)circle_size);
        debugmessage = desired_h_xdot;
    } else {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        circle_test_did_start = false;
        timer_set = false;
        debugmessage = 3;
    } 
}

void sinusoid_test() {
  int curve_size = 2;
  if (millis() < start_test_time + 5000) {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
    } else if (millis() < start_test_time + 5000 + 6282*curve_size) {
        seconds = (float)(millis() - start_test_time-5000)/1000.0;
        R = .2;
        desired_h_xdot = -R * sin(seconds/(float)curve_size);  //6.28*curve_size second revolution
        desired_h_ydot = R;  
    } else {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        sine_test_did_start = false;
        timer_set = false;
    }
}

void spiral_test() {
    if (millis() < start_test_time + 5000) {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
    } else if (
        millis() < start_test_time + 35000) {
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
    if (millis() < start_test_time + 5000) {
        desired_h_xdot = 0;
        desired_h_ydot = 0;
        debugmessage = 1;
    } else if (millis() < start_test_time + 5000 + circle_size*6282) {
        seconds = (float)(millis() - start_test_time-5000)/1000.0;
        R = .2;    
        desired_h_xdot = 0;  //6.28*circle_size second revolution
        desired_h_ydot = .2;
        desired_h_rdot = (float)6.282/(((float)circle_size * 6282)/1000);
        debugmessage = desired_h_xdot;
    } else {
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
