#include "global_variables.h"

//one thing to note is that I might want to include a global variable as to whether or not the robo will veer away from a wall

int too_close_distance = 4; //set this variable later to whatever the ultrasonic sensors output, in centimeters

int veer_away_from_wall(int direction)
{
  //averages distances on sides
  
  double distance_side[4] = new double[];
  for(int i = 0; i < 4; i++)
  {
    distance_side[i] = (sensor_values[i * 2] + sensor_values[i * 2 + 1]);
  }
  
  int veer_direction;
  
  for(int i = 0; i < 4; i++)
  {
    if(distance_side[i] < too_close_distance)
        veer_direction = i;
  }
  
  if(veer_direction == null)
    veer_direction = -1;
    
  return veer_direction * 2;
}
