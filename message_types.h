#ifndef MESSAGE_TYPES_H
#define MESSAGE_TYPES_H

const uint8 HolonomicVelocityMessage = 103;
typedef struct __attribute__((__packed__)) {
    uint8_t type;
    float x_dot;
    float y_dot;
    float r_dot;
} HolonomicVelocity;


#endif

