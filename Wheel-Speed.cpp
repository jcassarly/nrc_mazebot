#include "global.h"

void setSpeed(int direction, int speed, int adjustment) //adjustment is for getParallel, which would calculate how much adjustment is needed
{
  //assuming pins are 1,2,3,4.  these will be changed later.  AnalogWrite takes (pin number, speed (0 - 255))
  if(direction == 0) //"forward"
  {
   analogWrite(1, speed + adjustment);
   analogWrite(2, speed + adjustment);
   analogWrite(3, -speed + adjustment);
   analogWrite(4, -speed + adjustment);
  }
  else if(direction == 2) //"right"
  {
   analogWrite(1, =speed + adjustment);
   analogWrite(2, speed + adjustment);
   analogWrite(3, speed + adjustment);
   analogWrite(4, -speed + adjustment);
  }
  else if(direction == 4) //left
  {
   analogWrite(1, speed + adjustment);
   analogWrite(2, -speed + adjustment);
   analogWrite(3, -speed + adjustment);
   analogWrite(4, speed + adjustment); 
  }
  else if(direction == 6) //backwards
  {
   analogWrite(1, -speed + adjustment);
   analogWrite(2, -speed + adjustment);
   analogWrite(3, speed + adjustment);
   analogWrite(4, speed + adjustment); 
  }
               
}
