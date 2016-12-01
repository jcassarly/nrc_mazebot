# nrc_mazebot

Source code for the Case Western 2016 NRC Mazebot

##Pseudocode

###Main Function

Assigned to Jared
    
    Check Distances
    while (not at the end of the maze)
        determine direction and go
        while (not at a turn)
            check distance
            veer away from wall
            get parallel

###Direction

####Determine the Direction

Assigned to Megan

Decide based on sensor value where to send the robot<br />
Set a global variable that indicates the direction the robot should be moving

###Wheels

####Define Wheels and Directions

Assigned to ?

       a2___3b 
       1/ * \4 
       .|   | 
      d8\___/5c 
       7     6 

*arduino is here


####Set Wheel Speeds

Assigned to Jonathan

Set all the wheels to some speed<br />
2 are negative that value

    void move_direction(direction, speed)
        if direction == forward
            wheel_a = speed
            wheel_b = speed
            wheel_c = -speed
            wheel_d = -speed
        else if direction == right
            wheel_a = -speed
            wheel_b = speed
            wheel_c = speed
            wheel_d = -speed
        else if direction == left
            wheel_a = speed
            wheel_b = -speed
            wheel_c = -speed
            wheel_d = speed
        else if direction == backward
            wheel_a = -speed
            wheel_b = -speed
            wheel_c = speed
            wheel_d = speed

###Normal Walls

####Get Distance From Wall

Assigned to Adam/Andrea

Get the value from an ultrasonic sensor that tells how far it is form a wall

    double[] sensor_values = 8 double
    
    void get_distance()
        for each sensor_value (done in 2 groups of 4)
            get the reading from it
            set the corresponding value in the sensor_values array to that value (first round do evens, then do odds to show that sensor values alternate)
        check for angled wall?
        check for ramp?


####Get Parallel to Wall

Assigned to Jared

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

            when robot is done rotating, undo the speed change
        else 
            everythings good

####Rotate

Assigned to Kate

Rotate the robot by some speed change

    rotate(speed_change)
        for each motor
            selected_motor_speed += speed_change

Causes robot to change direction.<br />
Some wheels will have an equivalent negative value.<br />
Wheels with positive values speed up, wheels with negative values slow down


####Veer Away From Wall

Assigned to Jonathan

When a robot gets too close to a wall, robot rotates to move away from the wall. <br />
This function takes higher precedence than getParallel

###Angled Wall

Assigned to Megan

If there is a significant difference between differences in front and back, there’s an angled wall. <br />
Dont use angled wall to keep parallel

###Ramps

Assigned to Adam/Andrea

Use opposite side sensors until ramp is detected<br />
If gyroscope detects ramp, start using side walls to keep parallel

if we are going down the ramp we should slow down<br />
if we are going up the ramp, we should speed up

##Things from the design review to consider
ultrasonic sensors are slow - anno has an idea on how to make it faster

when looking to take turns, if there is one, take it (this might be what we already have idk)

thought completely autonomous would be easier to code

##Link to Mobile Mapping Repository
https://github.com/Spwizzard/MobileMapping