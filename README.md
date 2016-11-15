# nrc_mazebot

Source code for the Case Western 2016 NRC Mazebot

##Pseudocode

####getParalell(side)
// Define epsilon: threshold for 

get distance from wall for each relevant sensor

Take absolute value of the diifference and check if less than epsilon
int diff = |sensor_value_1 - sensor_value_2|
if diff < epsilon
    choose larger value and rotate towards it
else 
    everythings good


####getDistance()