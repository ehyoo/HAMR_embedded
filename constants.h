#ifndef CONSTANTS_h
#define CONSTANTS_h
#define M1_FORWARD 1
#define M2_FORWARD 1
#define MT_COUNTER 0
/* interrupt pins */
#define PIN_MT_ENCODER_A 62
#define PIN_MT_ENCODER_B 63

/*
 * Robot Physical Constants. MODIFY THESE TO REFLECT NEW ROBOT
 */
#define TICKS_PER_REV_DDRIVE  16383.0                  // number of encoder ticks in one full rotation on diff drive motor
#define TICKS_PER_REV_TURRET  16383.0 * (225.0/24.0)*16.0  // actual measured 1024.0 * (225/24) (16383 the new encoder, 225/24 being the other gear, 16 being the turret motor gearbox). Note this was 220/24 for some reason.
#define WHEEL_DIAMETER        0.12192                 // actually 4.85 ' diameter // in meters (4.8' diameter)  
#define WHEEL_RADIUS          (WHEEL_DIAMETER / 2.0)  // wheel radius, in meters
#define WHEEL_DIST            0.270413 //0.328168                // distance between diff drive wheels, in meters (12.92')
#define DIST_PER_REV          (PI*WHEEL_DIAMETER)     // circumference of wheel in meters
#define LOOPTIME              1000                    // Desired speed of the main loop in microseconds
#define DEBUGTIME             100000                    // Desired speed of the main loop in microseconds


/**
 * Pin Definitions
 * M1 = Right motor. actually i think this is left: andrew
 * M2 = Left motor. actuall i think this is right: andrew
 * MT = Turret motor. cw is positive.
 */
/* Motor Driver Pinouts */
#define M2_PWM_PIN 5
#define M2_DIR_PIN 0

#define M1_PWM_PIN 4
#define M1_DIR_PIN 1

#define MT_PWM_PIN 3
#define MT_DIR_PIN 2

#endif



