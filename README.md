# nrc_mazebot

Source code for the Case Western 2016 NRC Mazebot

##Pseudocode

###Normal Walls

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
        if difference > epsilon
            choose larger value and rotate towards it
                rotate the robot until difference is less than epsilon
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
Wheels with positive values speed up, wheels with negative values slow down


####Veer Away From Wall

When a robot gets too close to a wall, robot rotates to move away from the wall. <br />
This function takes higher precedence than getParallel

###Angled Wall

If there is a significant difference between differences in front and back, there’s an angled wall. <br />
Dont use angled wall to keep parallel

###Ramps

Use opposite side sensors until ramp is detected<br />
If gyroscope detects ramp, start using side walls to keep parallel

if we are going down the ramp we should slow down<br />
if we are going up the ramp, we should speed up

##Things from the design review to consider
ultrasonic sensors are slow - anno has an idea on how to make it faster

when looking to take turns, if there is one, take it (this might be what we already have idk)

thought completely autonomous would be easier to code