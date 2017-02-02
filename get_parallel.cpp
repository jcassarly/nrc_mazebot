#include "global_variables.h"
#include "get_parallel.h"
#include <math.h>

double parallel_epsilon = 0; // value that is the difference between the sensor values is greater than this, the robot is not parallel
double angled_epsilon = 0; // value that if the difference between the sensor values is greater than this, an angled wall is here
double speed_change = 0;

double get_parallel(int side) {
    double sensor1 = sensor_values[side];
    double sensor2 = sensor_values[side + 1];
    double diff = abs(sensor2 - sensor1);

    // if the robot is on a ramp, look at a wall to the side
    if (on_ramp) {
        sensor1 = sensor_values[(8 - side + 2) % 8];
        sensor2 = sensor_values[(8 - side + 2) % 8];
        diff = abs(sensor2 - sensor1);
    }
    // if the wall in the current direction is angled, switch to the wall behind
    else if (diff > angled_epsilon) {
        sensor1 = sensor_values[(8 - side + 4) % 8];
        sensor2 = sensor_values[(8 - side + 4) % 8];
        diff = abs(sensor2 - sensor1);
    }

    // if the difference between the sensor values is too large and the robot is not getting parallel
    if (diff > parallel_epsilon && !getting_parallel) {
        // if the first sensor is larger, rotate to make the sensor1 smaller
        if (sensor1 > sensor2) {
            return -speed_change;
        }
        // if the second sensor is larger, rotate to make sensor2 smaller
        else {
            return speed_change;
        }
        getting_parallel = true;
    }
    // if the difference between the sensor values is too large and the robot is getting parallel
    else if (diff > parallel_epsilon && getting_parallel) {
        // keep doing what its doing
        getting_parallel = true;
    }
    else {
        // stop rotating
        return 0;
        getting_parallel = false;
    }
}

