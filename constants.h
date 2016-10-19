#ifndef CONSTANTS_h
#define CONSTANTS_h


#define M1_FORWARD 1
#define M2_FORWARD 1
#define MT_COUNTER 0
/* interrupt pins */
#define PIN_MT_ENCODER_A 62
#define PIN_MT_ENCODER_B 63
/*
 * Robot Constants. MODIFY THESE TO REFLECT NEW ROBOT
 */
#define TICKS_PER_REV_DDRIVE  4096.0                  // number of encoder ticks in one full rotation on diff drive motor
#define TICKS_PER_REV_TURRET  1024.0 * (220.0/24.0)*(1.0+8.0/360.0)  // actual measured 1024.0 * (220/24) (changed by CRI project- 1024 the new encoder, 220/24 being the other gear)
#define WHEEL_DIAMETER        0.12192                 // actually 4.85 ' diameter // in meters (4.8' diameter)  
#define WHEEL_RADIUS          (WHEEL_DIAMETER / 2.0)  // wheel radius, in meters
#define WHEEL_DIST            0.328168                // distance between diff drive wheels, in meters (12.92')
#define DIST_PER_REV          (PI*WHEEL_DIAMETER)     // circumference of wheel in meters
#define LOOPTIME              10.0 // in ms

// For live-plotting
/* SIGNALS */
#define SIG_START_STRING '$'
#define SIG_START_LOG '['
#define SIG_STOP_LOG ']'
#define SIG_UNINITIALIZED '!'

#define SIG_HOLO_X 'x'
#define SIG_HOLO_Y 'y'
#define SIG_HOLO_R 'a'

#define SIG_DD_V 'd'
#define SIG_DD_R 'D'

#define SIG_R_MOTOR 'r'
#define SIG_L_MOTOR 'l'
#define SIG_T_MOTOR 't'

#define SIG_R_KP '1'
#define SIG_R_KI '2'
#define SIG_R_KD '3'

#define SIG_L_KP '4'
#define SIG_L_KI '5'
#define SIG_L_KD '6'

#define SIG_T_KP '7'
#define SIG_T_KI '8'
#define SIG_T_KD '9'

#define SIG_HOLO_X_KP 'Q'
#define SIG_HOLO_X_KI 'W'
#define SIG_HOLO_X_KD 'E'

#define SIG_HOLO_Y_KP 'R'
#define SIG_HOLO_Y_KI 'T'
#define SIG_HOLO_Y_KD 'Y'

#define SIG_HOLO_R_KP 'U'
#define SIG_HOLO_R_KI 'I'
#define SIG_HOLO_R_KD 'O'

#define MSG_REQUEST 'M'

#define SIG_MOVE_FALSE 'z'


/********************* *************************
 * Pin Definitions
 * 
 * M1 = Left Differential Drive (ddrive) motor 
 * M2 = Right Differential Drive (ddrive) motor
 * MT = Turret motor
 **********************************************/
/* Motor Driver Pinouts */
#define M1_PWM_PIN 5
#define M1_DIR_PIN 4
// #define M1_SLP_PIN 45
// #define M1_FLT_PIN 47

#define M2_PWM_PIN 7
#define M2_DIR_PIN 6
// #define M2_SLP_PIN 51
// #define M2_FLT_PIN 53

#define MT_PWM_PIN 12
#define MT_DIR_PIN 13
// #define MT_SLP_PIN 39
// #define MT_FLT_PIN 41


/* Decoder Pinouts */
// #define DECODER_SEL_PIN 48
// #define DECODER_OE_PIN 50
// #define DECODER_RST_PIN 52

const int M1_DECODER_D_PINS[8] = {19,18,17,16,15,14,24,22}; // D0-D7 pinouts
const int M2_DECODER_D_PINS[8] = {5,7,8,9,10,11,12,13}; // D0-D7 pinouts
const int MT_DECODER_D_PINS[8] = {44,42,40,38,36,34,32,30}; // D0-D7 pinouts

#endif

