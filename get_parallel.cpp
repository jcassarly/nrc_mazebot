#include "global_variables.h"
#include <math.h>

double parallel_epsilon = 0; // value that is the difference between the sensor values is greater than this, the robot is not parallel
double angled_epsilon = 0; // value that if the difference between the sensor values is greater than this, an angled wall is here
double speed_change = 0;

void get_parallel(int side) {
    double sensor1 = sensor_values[side];
    double sensor2 = sensor_values[side + 1];
    diff = abs(sensor2 - sensor1);

    // if the difference between the sensor values is too large and the robot is not getting parallel
    if (diff > parallel_epsilon && diff < angled_epsilon && !getting_parallel) {
        // if the first sensor is larger, rotate to make the sensor1 smaller
        if (sensor1 > sensor2) {
            move_direction(side, default_speed, speed_change);
        }
        // if the second sensor is larger, rotate to make sensor2 smaller
        else {
            move_direction(side, default_speed, speed_change);
        }
        getting_parallel = true;
    }
    // if the difference between the sensor values is too large and the robot is getting parallel
    else if (diff > parallel_epsilon && diff < angled_epsilon && getting_parallel) {
        // keep doing what its doing
        getting_parallel = true;
    }
    else {
        // stop rotating
        move_direction(side, default_speed, 0);
        getting_parallel = false;
    }
}

