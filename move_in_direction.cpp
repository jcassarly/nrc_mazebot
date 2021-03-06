#include "global_variables.h"
#include <Arduino.h>

int wheel1 = 1;
int wheel2 = 2;
int wheel3 = 3;
int wheel4 = 4;

final int THRESHOLD = 7; //modify this later to whatever the gyroscope gives.

void move_in_direction(int direction, int speed, int adjustment, int veer_direction, int16_t gyro_data) //adjustment is for getParallel, which would calculate how much adjustment is needed
{
    //assuming pins are 1,2,3,4.  these will be changed later.  AnalogWrite takes (pin number, speed (0 - 255))
    int gyro_modifier = 1;
    
    if(gyro_data > THRESHOLD)
        gyro_modifer = 2;
    
    
    if(veer_direction != -1)
    {
        if(veer_direction == 0)
        {
            analogWrite(wheel1, -speed);
            analogWrite(wheel2, -speed);
            analogWrite(wheel3, speed);
            analogWrite(wheel4, speed);
        }
        
        else if(veer_direction == 2)
        {
            analogWrite(wheel1, speed);
            analogWrite(wheel2, -speed);
            analogWrite(wheel3, -speed);
            analogWrite(wheel4, speed);
        }

        else if(veer_direction == 4)
        {
            analogWrite(wheel1, -speed);
            analogWrite(wheel2, speed);
            analogWrite(wheel3, speed);
            analogWrite(wheel4, -speed);
        }

        else if(veer_direction == 6)
        {
            analogWrite(wheel1, speed);
            analogWrite(wheel2, speed);
            analogWrite(wheel3, -speed);
            analogWrite(wheel4, -speed);
        }   
    }

    else if(direction == 0) //"forward"
    {
        analogWrite(wheel1, (speed + adjustment)/gyro_modifier);
        analogWrite(wheel2, (speed + adjustment)/gyro_modifier);
        analogWrite(wheel3, (-speed + adjustment)/gyro_modifier);
        analogWrite(wheel4, (-speed + adjustment)/gyro_modifier);
    }

    else if(direction == 2) //"right"
    {
        analogWrite(wheel1, (-speed + adjustment)/gyro_modifier);
        analogWrite(wheel2, (speed + adjustment)/gyro_modifier);
        analogWrite(wheel3, (speed + adjustment)/gyro_modifier);
        analogWrite(wheel4, (-speed + adjustment)/gyro_modifier);
    }

    else if(direction == 4) //left
    {
        analogWrite(wheel1, (speed + adjustment)/gyro_modifier);
        analogWrite(wheel2, (-speed + adjustment)/gyro_modifier);
        analogWrite(wheel3, (-speed + adjustment)/gyro_modifier);
        analogWrite(wheel4, (speed + adjustment)/gyro_modifier); 
    }

    else if(direction == 6) //backwards
    {
        analogWrite(wheel1, (-speed + adjustment)/gyro_modifier);
        analogWrite(wheel2, (-speed + adjustment)/gyro_modifier);
        analogWrite(wheel3, (speed + adjustment)/gyro_modifier);
        analogWrite(wheel4, (speed + adjustment)/gyro_modifier); 
    }
               
}
