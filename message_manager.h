#ifndef MESSAGE_MANAGER_H
#define MESSAGE_MANAGER_H

#include "message_types.h"

typedef struct __attribute__ ((__packed__)) {
    HolonomicVelocity holo_vel_struct;
} MESSAGE_MANAGER_t;

#endif

