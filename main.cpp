
#include <math.h>
// put these in the same folder
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"

void analogWrite(int a, int b); 
void get_distance();

void get_direction(int prev_dir);
float get_parallel(int side);
void move_in_direction(int direction, int speed, int adjustment, int veer_direction);
int veer_away_from_wall(int direction);

void analogWrite( int a, int b) {

}

void get_distance() {

}

/*GLOBAL VARIABLES*/

#define RIGHT 2
#define LEFT 6
#define UP 0
#define DOWN 4

#define DONE 8

#define RAMP_EPSILON 12
#define PARALLEL_EPSILON 0.08 // value that is the difference between the sensor values is greater than this, the robot is not parallel
#define ANGLED_EPSILON 18.08 // value that if the difference between the sensor values is greater than this, an angled wall is here

#define DEFAULT_SPEED 150
#define SPEED_CHANGE 30

// the current direction the robot is moving (values 1-4 - see .h file for declarations)
int direction = RIGHT; // this should be set in the direction facing the wall on the opposite side that we are going

// distances the sensors pick up in centimeters
float sensor_values[8] = {0}; 

// wheel speeds correspodning to letter of variable (see README for which letter is which)
Adafruit_DCMotor *frontL = AFMS.getMotor(1);
Adafruit_DCMotor *frontR = AFMS.getMotor(4);
Adafruit_DCMotor *rearL  = AFMS.getMotor(2);
Adafruit_DCMotor *rearR  = AFMS.getMotor(3);

// tells whether the robot is currently trying to get parallel to the wall
bool getting_parallel = false;

// 0 for not on a ramp, 1 for up, 2 for down
int which_ramp = 0;

// sets the angles for the robot
float default_angle = 0;

/*GET DIRECTION*/

#define E 10 //tbd epsilon value

void get_direction(int prev_dir){
    int back;
    int current;
    int max_val=E;
    int max_dir;

    //sets back to the direction we just came from
    if (prev_dir<4)
        back=prev_dir+4;
    else
        back=prev_dir-4;

    for (int i=0; i<8; i+=2){
        if (i!=back){ //doesn't check direction we just came from
            current=(int)(sensor_values[i]+sensor_values[i+1])/2; //sets current to the average of the two sensors pointing the same direction
            if (current > max_val){ //if the current distance is greater than the max distance, max_val becomes current and max_dir becomes the current direction being checked
                max_val=current;
                max_dir=i;
            }
        }
    }

    if (max_val!=E)
        direction=max_dir; //sets global variable to be the direction farthest away from a wall (the direction we need to move next)
    else
        direction=DONE; //we have reached the end of the maze if the farthest wall is within epsilon from the robot (not including direction we just came from)

}

/*GET PARALLEL*/

float get_parallel(int side) {
    float sensor1 = sensor_values[side];
    float sensor2 = sensor_values[side + 1];
    float diff = std::abs(sensor2 - sensor1);

    // if the robot is on a ramp, look at a wall to the side
    if (which_ramp > 0) {
        sensor1 = sensor_values[(8 - side - 2) % 8];
        sensor2 = sensor_values[(8 - side - 2) % 8];
        diff = std::abs(sensor2 - sensor1);
    }
    // if the wall in the current direction is angled, switch to the wall behind
    else if (diff > ANGLED_EPSILON) {
        sensor1 = sensor_values[(8 - side + 4) % 8];
        sensor2 = sensor_values[(8 - side + 4) % 8];
        diff = std::abs(sensor2 - sensor1);
    }

    // if the difference between the sensor values is too large and the robot is not getting parallel
    if (diff > PARALLEL_EPSILON && !getting_parallel) {
        // if the first sensor is larger, rotate to make the sensor1 smaller
        if (sensor1 > sensor2) {
            return -SPEED_CHANGE;
        }
        // if the second sensor is larger, rotate to make sensor2 smaller
        else {
            return SPEED_CHANGE;
        }
        getting_parallel = true;
    }
    // if the difference between the sensor values is too large and the robot is getting parallel
    else if (diff > PARALLEL_EPSILON && getting_parallel) {
        // keep doing what its doing
        getting_parallel = true;
    }
    else {
        // stop rotating
        return 0;
        getting_parallel = false;
    }
}

/*MOVE IN DIRECTION*/

void move_in_direction(int direction, uint8_t speed, uint8_t adjustment, int veer_direction) //adjustment is for getParallel, which would calculate how much adjustment is needed
{
    //assuming pins are 1,2,3,4.  these will be changed later.  AnalogWrite takes (pin number, speed (0 - 255))

    if(veer_direction >= 0)
    {
        if(veer_direction == 0) // forward
        {
            frontL->run(BACKWARD);
            frontR->run(FORWARD);
            rearL ->run(BACKWARD);
            rearR ->run(FORWARD);
        }
        
        else if(veer_direction == 2) // right
        {
            frontL->run(BACKWARD);
            frontR->run(BACKWARD);
            rearL ->run(FORWARD);
            rearR ->run(FORWARD);
        }

        else if(veer_direction == 4) // left
        {
            frontL->run(FORWARD);
            frontR->run(FORWARD);
            rearL ->run(BACKWARD);
            rearR ->run(BACKWARD);
        }

        else if(veer_direction == 6) // backwards
        {
            frontL->run(FORWARD);
            frontR->run(BACKWARD);
            rearL ->run(FORWARD);
            rearR ->run(BACKWARD);
        }   

        set_all_wheel_speeds(speed + adjustment);
    }

    else if(direction == 0) //"forward"
    {
        frontL->run(BACKWARD);
        frontR->run(FORWARD);
        rearL ->run(BACKWARD);
        rearR ->run(FORWARD);

        set_all_wheel_speeds(speed + adjustment);
    }

    else if(direction == 2) //"right"
    {
        frontL->run(BACKWARD);
        frontR->run(BACKWARD);
        rearL ->run(FORWARD);
        rearR ->run(FORWARD);

        set_all_wheel_speeds(speed + adjustment);
    }

    else if(direction == 4) //left
    {
        frontL->run(FORWARD);
        frontR->run(FORWARD);
        rearL ->run(BACKWARD);
        rearR ->run(BACKWARD);

        set_all_wheel_speeds(speed + adjustment);
    }

    else if(direction == 6) //backwards
    {
        frontL->run(FORWARD);
        frontR->run(BACKWARD);
        rearL ->run(FORWARD);
        rearR ->run(BACKWARD);

        set_all_wheel_speeds(speed + adjustment);
    }
               
}

void set_all_wheel_speeds(uint8_t speed) {
    frontL->setSpeed(speed);
    frontR->setSpeed(speed);
    rearL->setSpeed(speed);
    rearR->setSpeed(speed);
    delay(10);
}


/* VEER AWAY FROM WALL*/

//one thing to note is that I might want to include a global variable as to whether or not the robo will veer away from a wall

int too_close_distance = 4; //set this variable later to whatever the ultrasonic sensors output, in centimeters

int veer_away_from_wall(int direction)
{
    //averages distances on sides, and determines if one distance is less than the threshold value too_close_distance
    //stores direction that is below the threshold in veer direction

    int veer_direction = -1;

    for(int i = 0; i < 4; i++)
    {
        if(too_close_distance > (sensor_values[i * 2] + sensor_values[i * 2 + 1]))
            veer_direction = i;
    }

    return veer_direction * 2;
}

/*READ ACCEL*/

MPU6050 accelgyro;

float read_accelerometer() {
    // read in the raw x, y, and z values
    int16_t ax = accelgyro.getAccelerationX();
    int16_t ay = accelgyro.getAccelerationY();
    int16_t az = accelgyro.getAccelerationZ();

    // convert the raw data to degrees (get y angle only)
    return (180/3.141592) * atan(ay / sqrt(square(ax) + square(az)));
}

/*MAIN*/

void setup() {
    /* Set up Accelerometer */

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // read in the default angle
    default_angle = read_accelerometer();

    /* set up motors */

    AFMS.begin();  // create with the default frequency 1.6KHz
  
    // Set the speed to start, from 0 (off) to 255 (max speed)
    frontL->setSpeed(DEFAULT_SPEED);
    frontR->setSpeed(DEFAULT_SPEED);
    rearL ->setSpeed(DEFAULT_SPEED);
    rearR ->setSpeed(DEFAULT_SPEED);

    frontL->run(FORWARD);
    frontR->run(FORWARD);
    rearL ->run(FORWARD);
    rearR ->run(FORWARD);
    // turn on motor
    frontL->run(RELEASE);
    frontR->run(RELEASE);
    rearL ->run(RELEASE);
    rearR ->run(RELEASE);

    /* initialize some stuff */

    get_direction(direction);
}

void loop() {
    // as long as the robot is not done going through the maze
    while (direction != DONE) {
        
        float diff = read_accelerometer() - default_angle;
        if (std::abs(diff) > RAMP_EPSILON) {
            which_ramp = (diff > 0) ? 1 : 2;
        }
        else {
            which_ramp = 0;
        }

        // update the sensor values
        get_distance();

        if ((sensor_values[direction] + sensor_values[direction + 1]) / 2 < 10.25) { // edit to account for angled wall and maybe ramp
            // set the current direction
            get_direction(direction);
        }
        // move in that direction
        move_in_direction(direction, default_speed, get_parallel(direction), veer_away_from_wall(direction));

        delay(10);
    }

    // stop robot
    frontL->run(RELEASE);
    frontR->run(RELEASE);
    rearL ->run(RELEASE);
    rearR ->run(RELEASE);
    delay(1000);
}

