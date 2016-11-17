# nrc_mazebot

Source code for the Case Western 2016 NRC Mazebot

##Pseudocode

###Walls

####Get Parallel to Wall
// Define epsilon: threshold for 

get distance from wall for each relevant sensor (sensors of specified side)

Take absolute value of the diifference and check if less than epsilon

    get_parallel(side_of_robot)
        difference = |sensor_value_1 - sensor_value_2|
        if difference < epsilon
            choose larger value and rotate towards it
                rotate the robot until different is less than epsilon
                    larger value decreases
                    smaller value increases
        else 
            everythings good

####Rotate

    rotate(speed_change)
        for each motor
        adjust each motor by speed_change
            // causes robot to change direction
            // some wheels will have an equivalent negative value
                // wheels with positive values speed up,
                // wheels with negative values slow down

causes robot to change direction. 
\nsome wheels will have an equivalent negative value
\nwheels with positive values speed up,
\nwheels with negative values slow down


####Veer Away From Wall

###Ramps
can only use side walls for keeping parallel

if we are going down the ramp we should slow down

if we are going up the ramp, we should speed up

####Get Distance