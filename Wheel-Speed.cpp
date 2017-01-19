#include "global.h"

int wheel1 = 1;
int wheel2 = 2;
int wheel3 = 3;
int wheel4 = 4;

void setSpeed(int direction, int speed, int adjustment) //adjustment is for getParallel, which would calculate how much adjustment is needed
{
  //assuming pins are 1,2,3,4.  these will be changed later.  AnalogWrite takes (pin number, speed (0 - 255))
  if(direction == 0) //"forward"
  {
   analogWrite(wheel1, speed + adjustment);
   analogWrite(wheel2, speed + adjustment);
   analogWrite(wheel3, -speed + adjustment);
   analogWrite(wheel4, -speed + adjustment);
  }
  else if(direction == 2) //"right"
  {
   analogWrite(wheel1, =speed + adjustment);
   analogWrite(wheel2, speed + adjustment);
   analogWrite(wheel3, speed + adjustment);
   analogWrite(wheel4, -speed + adjustment);
  }
  else if(direction == 4) //left
  {
   analogWrite(wheel1, speed + adjustment);
   analogWrite(wheel2, -speed + adjustment);
   analogWrite(wheel3, -speed + adjustment);
   analogWrite(wheel4, speed + adjustment); 
  }
  else if(direction == 6) //backwards
  {
   analogWrite(wheel1, -speed + adjustment);
   analogWrite(wheel2, -speed + adjustment);
   analogWrite(wheel3, speed + adjustment);
   analogWrite(wheel4, speed + adjustment); 
  }
               
}
