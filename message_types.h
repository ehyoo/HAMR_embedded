#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

const uint8 HolonomicVelocityMessageType = 103;
typedef struct __attribute__((__packed__)) {
    uint8_t type;
    float x_dot;
    float y_dot;
    float r_dot;
} HolonomicVelocity;

const uint8 DifDriveVelocityMessageType = 104;
typedef struct __attribute__((__packed__)) {
  uint8_t type;
  float left_v;
  float right_v;
  float turret_v;
} DifDriveVelocity;

#endif


