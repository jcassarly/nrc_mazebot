# nrc_mazebot

Source code for the Case Western 2016 NRC Mazebot

##Pseudocode

###Walls

####Get Distance From Wall

Get the value from an ultrasonic sensor that tells how far it is form a wall

    get_distance(sensor)
        //TODO

####Get Parallel to Wall

get distance from wall for each relevant sensor (sensors of specified side) <br />
Take absolute value of the difference and check if less than epsilon <br />
Rotate robot if necessary

    #define epsilon __

    get_parallel(side_of_robot)
        sensor_value_1 = get_distance(right_sensor)
        sensor_value_2 = get_distance(left_sensor)
        difference = |sensor_value_1 - sensor_value_2|
        if difference < epsilon
            choose larger value and rotate towards it
                rotate the robot until different is less than epsilon
                    larger value decreases
                    smaller value increases
        else 
            everythings good

####Rotate

Rotate the robot by some speed change

    rotate(speed_change)
        for each motor
            selected_motor_speed += speed_change

Causes robot to change direction.<br />
Some wheels will have an equivalent negative value.<br />
Wheels with positive values speed up, wheels with negative values slow down<br />


####Veer Away From Wall

###Ramps
can only use side walls for keeping parallel

if we are going down the ramp we should slow down<br />
if we are going up the ramp, we should speed up<br />