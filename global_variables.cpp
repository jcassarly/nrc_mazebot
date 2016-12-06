#include "global_variables.h"

// the current direction the robot is moving (values 1-4 - see .h file for declarations)
int direction = RIGHT;

// distances the sensors pick up in centimeters
double sensor_values[8] = {0}; 

// wheel speeds correspodning to letter of variable (see README for which letter is which)
int wheel_a = 0;
int wheel_b = 0;
int wheel_c = 0;
int wheel_d = 0;