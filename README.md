# nrc_mazebot

Source code for the Case Western 2016 NRC Mazebot

##Pseudocode

####getParalell(side)
// Define epsilon: threshold for 

get distance from wall for each relevant sensor (sensors of specified side)

Take absolute value of the diifference and check if less than epsilon

    difference = |sensor_value_1 - sensor_value_2|
    if difference < epsilon
        choose larger value and rotate towards it
            large value decreases and smaller value decreases until deifference is less than epsilon

            to turn, you can adjust every motor by the same value by adding that value to its default move forward speed.
            by changing this speed, it will either speed up or slow down the motors and you will turn from the direction you were going.
            less tricky than slowing down specific motors and telling others to stay the same speed.
            going max clockwise is equal to some number, and going max counterclockwise is -1 * that number, so adding the same value to each
            motor's current motor speed should allow you to turn the robot easily.
    else 
        everythings good

###Keep Away From Wall


####getDistance()