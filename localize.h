#include "math.h"
#include "Arduino.h"

typedef struct location {
  // Current position
  float x;     // in meters
  float y;     // in meters
  float theta; // in rad

  // Current linear and angular
  float v; // linear velocity (m/s)
  float w; // angular velocity (rad/s)

  // Constructor
  location() { 
    x = 0.0;
    y = 0.0;
    theta = 0;
    v = 0.0;
    w = 0.0;
  }

  // Update location using encoder integration
  void update(float r_speed, float l_speed, float wheel_dist, float ts) {

    v = (l_speed + r_speed) / 2.0;
    w = (l_speed - r_speed) / wheel_dist;

    // Calculate velocities
    float ds = v * ts / 1000.0;
    float dtheta = - w * ts / 1000.0;

    // Update variables
    float dx = ds * cos(theta);
    float dy = ds * sin(theta);

    theta = theta + dtheta;

   if (theta > 2 * PI) {
     theta -= 2 * PI;
   }
   if (theta < 0) {
     theta += 2 * PI;
   }
     
    x = x + dx;
    y = y + dy;
  }
};



