#ifndef MAIN_H_
#define MAIN_H_

void get_direction(int prev_dir);
double get_parallel(int side);
void move_in_direction(int direction, int speed, int adjustment, int veer_direction);
int veer_away_from_wall(int direction);

#endif