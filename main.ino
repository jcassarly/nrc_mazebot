#include "global_variables.h"

setep() {

}

loop() {
    // as long as the robot is not done going through the maze
    while (direction != DONE) {
        // set the current direction
        get_direction(direction);
        // move in that direction
        move_direction(direction, default_speed, get_parallel(direction), veer_away_from_wall(direction));
    }

    // stop robot
    move_direction(direction, 0, 0, -1);
}