#include "global_variables.h"

int turn_epsilon = 0; // value that if the distance from the wall in the direction we are going is less than this, the robot needs to turn
int veer_epsilon = 0; // value that if the distance from a wall on the die of the robot is less than, the robot needs to veer awar from the wall

setep() {

}

loop() {
    while (!is_finished(direction)) {
        // set the current direction
        get_direction(direction);
        // move in that direction
        move_direction(direction, default_speed, 0);

        while (get_lesser_distance(direction) > turn_epsilon) {
            // if the lesser distance from one wall to the side is too close to the wall
            if (get_lesser_distance((8 - direction + 2) % 8) < veer_epsilon) {
                // veer away from said wall
                veer_away_from_wall((8 - direction + 2) % 8);
            }
            // if the distance from one wall to the other side is too close to the wall
            else if (get_lesser_distance((8 - direction - 2) % 8) < veer_epsilon) {
                // veer away from said wall
                veer_away_from_wall((8 - direction - 2) % 8)
            }

            // figure out how to handle angled wall tho, this line wont be enough
            get_parallel(direction);
        }
    }

    // set wheel speeds to 0
    move_direction(direction, 0, 0);
}

// gets the lesser distance from the wall in a given direction
int get_lesser_distance(int direction) {
    // returns the lesser of the 2 sensor_values for a given side
    if (sensor_values[direction] > sensor_values[direction + 1]) {
        return sensor_values[direction + 1];
    }
    else {
        return sensor_values[direction];
    }
}

// checks if the robot is finished in the maze
bool is_finished(direction) {
    if (get_lesser_distance(direction < turn_epsilon) && get_lesser_distance((8 - direction + 2) % 8) < turn_epsilon && get_lesser_distance((8 - direction - 2) % 8) < turn_epsilon) {
        return true;
    }
    else {
        return false;
    }
}