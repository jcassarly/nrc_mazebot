#include "global_variables.h"

void get_direction(int prev_dir){
  int back;
  int current;
  int max_val=0;
  int max_dir;

  //sets back to the direction we just came from
  if (prev_dir<4)
    back=prev_dir+4;
  else
    back=prev_dir-4;

  //updates sensor_values array  
  get_distance();
  
  for (int i=0; i<8; i+=2){
    if (i!=back && i!=(back+1)){  //doesn't check direction we just came from
      current=(int)(sensor_values[i]+sensor_values[i+1])/2; //sets current to the average of the two sensors pointing the same direction
      if (current > max_val){ //if the current distance is greater than the max distance, max_val becomes current and max_dir becomes the current direction being checked
        max_val=current;
        max_dir=i;
      }
    }
  }

  direction=max_dir;  //sets global variable to be the direction farthest away from a wall (the direction we need to move next)
