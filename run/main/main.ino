
#include <math.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>
#include <LiquidCrystal.h>

//method declarations
void get_distance();
void get_direction(int prev_dir);
int8_t get_parallel(int side);
void move_in_direction(int direction, uint8_t speed, int8_t rotate_parallel, int veer_direction);
int veer_away_from_wall(int direction);
float read_accelerometer();
float analog_to_cm(int analog);
void set_all_wheel_speeds(uint8_t speed);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*GLOBAL VARIABLES - not get distance stuff*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define RIGHT 2
#define LEFT 6
#define FORWARDS 0
#define BACKWARDS 4

#define DONE 8

#define RAMP_EPSILON 14
#define PARALLEL_EPSILON 0.08 // value that is the difference between the sensor values is greater than this, the robot is not parallel
#define ANGLED_EPSILON 19.44 //19.44 value might be too high // value that if the difference between the sensor values is greater than this, an angled wall is here

#define DEFAULT_SPEED 150 // 150 -> max = 191
#define SPEED_CHANGE 50 // 50 -> max = 64
#define VEER_SPEED_CHANGE 50 // 50

#define WALL_EPSILON 12.7//10.25 // 4 inches
#define VEER_EPSILON 9
#define GET_PARALLEL_TO_WALL 30 // distance to start getting parallel to wall at
#define RAMP_BACK 40
#define RAMP_FRONT 25
#define RAMP_SIDE 17.78//right

#define DIST_SENSOR_TO_EDGE 4.3 // distance from the sensor to where the edge of the robot would be it it was a square
#define ANGLE_FACTOR 1.2247 // sin(15) + cos(15) (angles in degrees)
#define DIST_BETWEEN_SENSORS 17.78 // 7"

#define BIT_CONVERSION 0.0002156 // convert uint_8 * t to cm
#define TURN_2_DISTANCE 33.5 // was 33.75

#define DONE_EPSILON 16

#define RAMP 13 // 13
#define BEFORE_RAMP 12 // 12
#define SLOW 17
#define SECOND_ANGLE 16 //16

#define PRINT_STATE false

// the current direction the robot is moving 
int direction = FORWARDS; // this should be set in the direction facing the wall on the opposite side that we are going

// distances the sensors pick up in centimeters
float sensor_values[8] = {0}; 

// create motor shield
Adafruit_MotorShield AFMS; 
// wheel speeds correspodning to letter of variable (see README for which letter is which)
Adafruit_DCMotor *frontL;
Adafruit_DCMotor *frontR;
Adafruit_DCMotor *rearL;
Adafruit_DCMotor *rearR;

// 0 for not on a ramp, 1 for up, 2 for down
int which_ramp = 0;

// sets the angles for the robot
float default_angle = 0;

int turns = 0;
bool last = false;
bool first = true;

#define LED_1 24
#define LED_2 26
#define LED_3 28
#define LED_4 30
#define LED_5 32

#define LED_1_ON digitalWrite(LED_1, HIGH);
#define LED_2_ON digitalWrite(LED_2, HIGH);
#define LED_3_ON digitalWrite(LED_3, HIGH);
#define LED_4_ON digitalWrite(LED_4, HIGH);
#define LED_5_ON digitalWrite(LED_5, HIGH);

#define LED_1_OFF digitalWrite(LED_1, LOW);
#define LED_2_OFF digitalWrite(LED_2, LOW);
#define LED_3_OFF digitalWrite(LED_3, LOW);
#define LED_4_OFF digitalWrite(LED_4, LOW);
#define LED_5_OFF digitalWrite(LED_5, LOW);

/* LCD Vars */

//LCD STUFF
#define LCD_CONTRAST  39
#define LCD_RS        41
#define LCD_RW        43
#define LCD_EN        45
#define LCD_DB4       47
#define LCD_DB5       49
#define LCD_DB6       51
#define LCD_DB7       53

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);

/* GLOBAL VARIABLES - get distance stuff */

// Define the analog ports for the sensors
// these are correct as of February 25
// 0-3 are group A
// 4-7 are group B
#define ULT_0     A8
#define ULT_1     A9
#define ULT_2     A12
#define ULT_3     A13

#define ULT_4     A11
#define ULT_5     A10
#define ULT_6     A14
#define ULT_7     A15


// digital ports
#define RST_A_PIN   14
#define TRIG_A_PIN  15
#define NOR_A_PIN   16

#define RST_B_PIN   17
#define TRIG_B_PIN  18
#define NOR_B_PIN   19

#define THRESH      150
#define RST_DELAY   10

#define SAMPLING_INTERVAL 25

//MACROS
#define RESET_A digitalWrite(RST_A_PIN,HIGH); delay(RST_DELAY); digitalWrite(RST_A_PIN,LOW);
#define RESET_B digitalWrite(RST_B_PIN,HIGH); delay(RST_DELAY); digitalWrite(RST_B_PIN,LOW);
#define TRIG_A  digitalWrite(TRIG_A_PIN,HIGH); delayMicroseconds(10); digitalWrite(TRIG_A_PIN,LOW);
#define TRIG_B  digitalWrite(TRIG_B_PIN,HIGH); delayMicroseconds(10); digitalWrite(TRIG_B_PIN,LOW);

int sensorValue0, sensorValue1, sensorValue2, sensorValue3, 
sensorValue4, sensorValue5, sensorValue6, sensorValue7;

bool groupADone = false, groupBDone = true;

unsigned long lTime;
unsigned long lTimeLast;
unsigned long lDeltaT;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*GET DISTANCE*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *  this will apply some formula (yet undetermined) and return an answer in cm
 *  @return - a float value that represents the distance from the sensor to
 *  the nearest solid object in cm
 *  @param - an int value that is the analog 10-bit analog value read from 
 *  the sensors.
 */
float analog_to_cm(int analog){
  return (.0581 * analog) - 13.309;
}

/**
 *  This function populates the global array "sensor_values" with cm distance
 *  values for each of the 8 sensors (0-7).
 */
bool get_distance(unsigned long t){
    bool norA  = digitalRead(NOR_A_PIN);
    bool norB  = digitalRead(NOR_B_PIN);

    // 0 % 2 = 0, but this doesn't guarantee enough time
    //unsigned long mod = (t>=15 ? (t/15)%2 : 1);

    if(norA && groupBDone && t>=SAMPLING_INTERVAL) { 
        sensorValue0 = analogRead(ULT_0);//2
        sensorValue1 = analogRead(ULT_1);//4
        sensorValue2 = analogRead(ULT_2);//6
        sensorValue3 = analogRead(ULT_3);//0
        (sensorValue0 > THRESH ? sensor_values[2] = analog_to_cm(sensorValue0) : sensor_values[2] = sensor_values[2]);          // if not above THESH, then sensor hasn't yet returned a result
        (sensorValue1 > THRESH ? sensor_values[4] = analog_to_cm(sensorValue1) : sensor_values[4] = sensor_values[4]); 
        (sensorValue2 > THRESH ? sensor_values[6] = analog_to_cm(sensorValue2) : sensor_values[6] = sensor_values[6]);
        (sensorValue3 > THRESH ? sensor_values[0] = analog_to_cm(sensorValue3) : sensor_values[0] = sensor_values[0]);

        RESET_A
        TRIG_A

        groupADone = true;              // toggle bool flags
        groupBDone = false;             // 
        
        return true;

    }
    else if(norB && groupADone && t>=SAMPLING_INTERVAL) {
        sensorValue4 = analogRead(ULT_4);//7
        sensorValue5 = analogRead(ULT_5);//5
        sensorValue6 = analogRead(ULT_6);//3
        sensorValue7 = analogRead(ULT_7);//1
        (sensorValue4 > THRESH ? sensor_values[7] = analog_to_cm(sensorValue4) : sensor_values[7] = sensor_values[7]);          // if not above THRESH, then sensor hasn't yet returned a result
        (sensorValue5 > THRESH ? sensor_values[5] = analog_to_cm(sensorValue5) : sensor_values[5] = sensor_values[5]); 
        (sensorValue6 > THRESH ? sensor_values[3] = analog_to_cm(sensorValue6) : sensor_values[3] = sensor_values[3]);
        (sensorValue7 > THRESH ? sensor_values[1] = analog_to_cm(sensorValue7) : sensor_values[1] = sensor_values[1]);

        RESET_B
        TRIG_B

        groupBDone = true;
        groupADone = false;
        
        return true;

    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*GET DIRECTION*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void get_direction(int prev_dir){
    int back;
    int current;
    int max_val=DONE_EPSILON;
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
                Serial.println(current);
                Serial.println(max_dir);
            }
        }
    }

    if (max_val > DONE_EPSILON)
        direction=max_dir; //sets global variable to be the direction farthest away from a wall (the direction we need to move next)
    else
        direction=DONE; //we have reached the end of the maze if the farthest wall is within epsilon from the robot (not including direction we just came from)

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*GET PARALLEL*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns a positive or negative value that is used by move_in_direction to rotate the robot so that it gets parallel to the wall
 * @param side the wall the robot is looking at and needs to get parallel to
 * @return the positive or negative value to make the robot get parallel to the wall.  Return 0 if the robot is parallel.
 */
int8_t get_parallel(int side) {
    //LED_1_ON
    float sensor1 = sensor_values[side];
    float sensor2 = sensor_values[side + 1];
    float diff = fabs(sensor2 - sensor1);

    // if the robot is on a ramp, look at a wall to the side
    /*if (turns == RAMP) {
        sensor1 = sensor_values[(side + 6) % 8];
        sensor2 = sensor_values[(side + 6) % 8];
        diff = fabs(sensor2 - sensor1);
    }
    // if the wall in the current direction is angled, switch to the wall behind
    else*/ if (diff * ANGLE_FACTOR > ANGLED_EPSILON) {
        sensor1 = sensor_values[(side + 4) % 8];
        sensor2 = sensor_values[(side + 4) % 8];
        diff = fabs(sensor2 - sensor1);
    }

    // if the difference between the sensor values is too large
    if (diff > PARALLEL_EPSILON) {
        // if the first sensor is larger, rotate to make the sensor1 smaller
        if (sensor1 > sensor2) {
            return SPEED_CHANGE;
        }
        // if the second sensor is larger, rotate to make sensor2 smaller
        else {
            return -SPEED_CHANGE;
        }
        
    }
    else {
        // stop rotating
        return 0;
        
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*MOVE IN DIRECTION*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void move_in_direction(int direction, uint8_t speed, int8_t rotate_parallel, int veer_direction) //adjustment is for getParallel, which would calculate how much adjustment is needed
{
    //printLCD((float) speed);
    //LED_1_OFF
    //LED_2_OFF
    if(direction == FORWARDS) //"forward"
    {
        frontL->run(BACKWARD);
        frontR->run(FORWARD);
        rearL ->run(BACKWARD);
        rearR ->run(FORWARD);

        if (veer_direction < 0) {
            if (rotate_parallel > 0) {
                printLCD_state("PARALLEL");
                // increase backward speed
                frontL->setSpeed(speed + SPEED_CHANGE);
                frontR->setSpeed(speed - SPEED_CHANGE);
                rearL ->setSpeed(speed + SPEED_CHANGE);
                rearR ->setSpeed(speed - SPEED_CHANGE);
            }
            else if (rotate_parallel < 0) {
                printLCD_state("PARALLEL");
                // increase forawrd speed
                frontL->setSpeed(speed - SPEED_CHANGE);
                frontR->setSpeed(speed + SPEED_CHANGE);
                rearL ->setSpeed(speed - SPEED_CHANGE);
                rearR ->setSpeed(speed + SPEED_CHANGE);
            } 
            else {
                printLCD_state("STRAIGHT");
                // go at speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed);
            }
        }
        else {
            if (veer_direction == (direction + 2) % 8) {

                printLCD_state("Veering from right");
                // increase forward speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                rearL ->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                rearR ->setSpeed(speed);
            }
            else if (veer_direction == (direction + 6) % 8) {
                printLCD_state("Veering from left");
                // increase backward speed
                frontL->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
            }
            else {
                printLCD_state("Weird mode");
                // go at speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed);
            }
            
        }
    }

    else if(direction == RIGHT) //"right"
    {
        frontL->run(BACKWARD);
        frontR->run(BACKWARD);
        rearL ->run(FORWARD);
        rearR ->run(FORWARD);

        if (veer_direction < 0) {
            if (rotate_parallel > 0) {
                printLCD_state("PARALLEL");
                // increase backward speed
                frontL->setSpeed(speed + SPEED_CHANGE);
                frontR->setSpeed(speed + SPEED_CHANGE);
                rearL ->setSpeed(speed - SPEED_CHANGE);
                rearR ->setSpeed(speed - SPEED_CHANGE);
            }
            else if (rotate_parallel < 0) {
                printLCD_state("PARALLEL");
                // increase forawrd speed
                frontL->setSpeed(speed - SPEED_CHANGE);
                frontR->setSpeed(speed - SPEED_CHANGE);
                rearL ->setSpeed(speed + SPEED_CHANGE);
                rearR ->setSpeed(speed + SPEED_CHANGE);
            } 
            else {
                printLCD_state("STRAIGHT");
                // go at speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed);
            }
        }
        else {
            //printLCD("VEERING");
            if (veer_direction == (direction + 2) % 8) {
                printLCD_state("Veering from right");
                // increase foward speed
                frontL->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
            }
            else if (veer_direction == (direction + 6) % 8) {
                printLCD_state("Veering from left");
                // increase backward speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                rearL ->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                rearR ->setSpeed(speed);
            }
            else {
                printLCD_state("Weird mode");
                // go at speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed);
            }
            
            
        }
    }

    else if(direction == LEFT) //left
    {
        frontL->run(FORWARD);
        frontR->run(FORWARD);
        rearL ->run(BACKWARD);
        rearR ->run(BACKWARD);

        if (veer_direction < 0) {
            if (rotate_parallel > 0) {
                printLCD_state("PARALLEL");
                // increase backward speed
                frontL->setSpeed(speed - SPEED_CHANGE);
                frontR->setSpeed(speed - SPEED_CHANGE);
                rearL ->setSpeed(speed + SPEED_CHANGE);
                rearR ->setSpeed(speed + SPEED_CHANGE);
            }
            else if (rotate_parallel < 0) {
                printLCD_state("PARALLEL");
                // increase forawrd speed
                frontL->setSpeed(speed + SPEED_CHANGE);
                frontR->setSpeed(speed + SPEED_CHANGE);
                rearL ->setSpeed(speed - SPEED_CHANGE);
                rearR ->setSpeed(speed - SPEED_CHANGE);
            } 
            else {
                printLCD_state("STRAIGHT");
                // go at speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed);
            }
        }
        else {
            //printLCD("VEERING");
            if (veer_direction == (direction + 2) % 8) {
                printLCD_state("Veering from right");
                // increase foward speed
                frontL->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
            }
            else if (veer_direction == (direction + 6) % 8) {
                printLCD_state("Veering from left");
                // increase backward speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                rearL ->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                rearR ->setSpeed(speed);
            }
            else {
                printLCD_state("Weird mode");
                // go at speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed);
            }
            
            
        }
    }

    else if(direction == BACKWARDS) //backwards
    {
        frontL->run(FORWARD);
        frontR->run(BACKWARD);
        rearL ->run(FORWARD);
        rearR ->run(BACKWARD);

        if (veer_direction < 0) {
            if (rotate_parallel > 0) {
                printLCD_state("PARALLEL");
                // increase backward speed
                frontL->setSpeed(speed - SPEED_CHANGE);
                frontR->setSpeed(speed + SPEED_CHANGE);
                rearL ->setSpeed(speed - SPEED_CHANGE);
                rearR ->setSpeed(speed + SPEED_CHANGE);
            }
            else if (rotate_parallel < 0) {
                printLCD_state("PARALLEL");
                // increase forawrd speed
                frontL->setSpeed(speed + SPEED_CHANGE);
                frontR->setSpeed(speed - SPEED_CHANGE);
                rearL ->setSpeed(speed + SPEED_CHANGE);
                rearR ->setSpeed(speed - SPEED_CHANGE);
            } 
            else {

                printLCD_state("STRAIGHT");
                // go at speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed);
            }
        }
        else {
            //printLCD("VEERING");
            if (veer_direction == (direction + 2) % 8) {
                printLCD_state("Veering from right");
                // increase foward speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                rearL ->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                rearR ->setSpeed(speed);
            }
            else if (veer_direction == (direction + 6) % 8) {
                printLCD_state("Veering from left");
                // increase backward speed
                frontL->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed + VEER_SPEED_CHANGE + ((turns == RAMP) ? 55 : 0));
            }
            else {
                printLCD_state("Weird mode");
                // go at speed
                frontL->setSpeed(speed);
                frontR->setSpeed(speed);
                rearL ->setSpeed(speed);
                rearR ->setSpeed(speed);
            }
            
        }
    }
               
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////]



/* VEER AWAY FROM WALL*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//one thing to note is that I might want to include a global variable as to whether or not the robo will veer away from a wall

int veer_away_from_wall(int direction)
{
    //LED_2_ON
    //averages distances on sides, and determines if one distance is less than the threshold value too_close_distance
    //stores direction that is below the threshold in veer direction

    int veer_direction = -1;
    float smaller_value_1 = (sensor_values[(direction + 2) % 8] < sensor_values[((direction + 2) % 8) + 1]) ? sensor_values[(direction + 2) % 8] : sensor_values[((direction + 2) % 8) + 1];
    float smaller_value_2 = (sensor_values[(direction + 6) % 8] < sensor_values[((direction + 6) % 8) + 1]) ? sensor_values[(direction + 6) % 8] : sensor_values[((direction + 6) % 8) + 1];
    if (smaller_value_1 > smaller_value_2) {
        if (smaller_value_2 < (turns == RAMP) ? WALL_EPSILON + 2.54 : VEER_EPSILON) veer_direction = (direction + 6) % 8;
    }
    else {
      if (smaller_value_1 < (turns == RAMP) ? WALL_EPSILON + 2.54 : VEER_EPSILON) veer_direction = (direction + 2) % 8;
    }

    return veer_direction;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////]



/* WADDLE UP RAMP*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void drive_ramp(int veer) {
    frontL->run(BACKWARD);
    frontR->run(BACKWARD);
    rearL ->run(FORWARD);
    rearR ->run(FORWARD);
  
    if (last) {
        frontL->setSpeed(DEFAULT_SPEED + ((veer == (direction + 6) % 8) ? 50 : 0));
        frontR->setSpeed(DEFAULT_SPEED + ((veer == (direction + 6) % 8) ? 50 : 0));
        rearL ->setSpeed(0);
        rearR ->setSpeed(0);
        /*while (fabs(sensor_values[(direction + 6) % 8] - sensor_values[((direction + 6) % 8) + 1]) > PARALLEL_EPSILON) {
            updateSensors();
        }*/
        delay((veer == (direction + 6) % 8) ? 100 : 100);
        
    }
    else {
        frontL->setSpeed(0);
        frontR->setSpeed(0);
        rearL ->setSpeed(DEFAULT_SPEED + ((veer == (direction + 2) % 8) ? 50 : 0));
        rearR ->setSpeed(DEFAULT_SPEED + ((veer == (direction + 2) % 8) ? 50 : 0));
        /*while (fabs(sensor_values[(direction + 2) % 8] - sensor_values[((direction + 2) % 8) + 1]) > PARALLEL_EPSILON) {
            updateSensors();
        }*/
        int dly = 100;
        if (first) {
            dly = 50;
        }
        else if (veer == (direction + 2) % 8) {
            dly = 100;
        }
        delay(dly);
    }

    frontL->run(RELEASE);
    frontR->run(RELEASE);
    rearL ->run(RELEASE);
    rearR ->run(RELEASE);
    delay(500);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*READ ACCELEROMETER*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MPU6050 accelgyro;

float read_accelerometer() {
    // read in the raw x, y, and z values
    int16_t ax = accelgyro.getAccelerationX();
    int16_t ay = accelgyro.getAccelerationY();
    int16_t az = accelgyro.getAccelerationZ();

    // convert the raw data to degrees (get y angle only)
    return (180/3.141592) * atan(sqrt(square(ay) + square(ax)) / az);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*MAIN*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printLCD(const char* msg) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(msg);
}

void printLCD_state(const char* msg) {
    if (PRINT_STATE) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(msg);
    }
}

void printLCD(float msg) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(msg);
}

void printLCD(int msg) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(msg);
}

void updateSensors() {
    lTime = millis();
    lDeltaT = lTime - lTimeLast;
    bool success = get_distance(lDeltaT);
    if(success){
        lTimeLast = lTime;
    }
}

void setup() {
    /* Set up Accelerometer */

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin(); // accelerometer is 0x40 (register 107) or 0x68 (register 117)

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

    /* set up ultrasonics */

    //Wire.begin(0x60); // motorshield between 0x60-0x80

    //Digital INPUTS
    pinMode(NOR_B_PIN, INPUT);
    pinMode(NOR_A_PIN, INPUT);


    //Analog INPUTS
    pinMode(ULT_0,INPUT);
    pinMode(ULT_1,INPUT);
    pinMode(ULT_2,INPUT);
    pinMode(ULT_3,INPUT);
    pinMode(ULT_4,INPUT);
    pinMode(ULT_5,INPUT);
    pinMode(ULT_6,INPUT);
    pinMode(ULT_7,INPUT);


    //Digital OUTPUTS
    //Initial state is LOW by default
    pinMode(RST_A_PIN, OUTPUT);
    pinMode(TRIG_A_PIN,OUTPUT);

    pinMode(RST_B_PIN, OUTPUT);
    pinMode(TRIG_B_PIN,OUTPUT);

    RESET_A
    RESET_B

    /* set up motors */
    AFMS = Adafruit_MotorShield();

    frontL = AFMS.getMotor(1);
    frontR = AFMS.getMotor(4);
    rearL  = AFMS.getMotor(2);
    rearR  = AFMS.getMotor(3);

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

    Serial.println("motor setup done");

    /* initialize some stuff */

    updateSensors();

    //LCD SETUP
    pinMode(LCD_RW,OUTPUT);
    digitalWrite(LCD_RW,LOW);
    pinMode(LCD_CONTRAST,OUTPUT);
    analogWrite(LCD_CONTRAST,50);
    lcd.begin(16, 2);
    lcd.print("Ready");

    pinMode(LED_1, INPUT);
    pinMode(LED_2, INPUT);
    pinMode(LED_3, INPUT);
    pinMode(LED_4, INPUT);
    pinMode(LED_5, INPUT);
}

void loop() {
    // Manual mode at the start before the first turn 
    printLCD("Manual - 1");
    updateSensors();
    get_direction(direction);
    float dist = (sensor_values[direction] + sensor_values[direction + 1]) / 2.0;
    while (dist > WALL_EPSILON) {
        updateSensors();
         
        dist = (sensor_values[direction] + sensor_values[direction + 1]) / 2.0;

        move_in_direction(direction, DEFAULT_SPEED, ((sensor_values[direction] + sensor_values[direction + 1]) / 2.0 < GET_PARALLEL_TO_WALL) ? get_parallel(direction) : 0, veer_away_from_wall(direction));

        delay(10);
    }
    delay(10);

    // Manual mode at the start between the first and second turns
    printLCD("Manual - 2");
    get_direction(direction);
    move_in_direction(direction, DEFAULT_SPEED, ((sensor_values[direction] + sensor_values[direction + 1]) / 2.0 < GET_PARALLEL_TO_WALL) ? get_parallel(direction) : 0, veer_away_from_wall(direction));
    dist = 0;
    updateSensors();
    
    float executions = 0;
    while (dist < TURN_2_DISTANCE) {

        dist = (sensor_values[(direction + 4) % 8] + sensor_values[((direction + 4) % 8) + 1]) / 2.0;
        
        move_in_direction(direction, DEFAULT_SPEED - 50, ((sensor_values[direction] + sensor_values[direction + 1]) / 2.0 < GET_PARALLEL_TO_WALL) ? get_parallel(direction) : 0, veer_away_from_wall(direction));

        //delay(10);
        updateSensors();
        executions += 1.0;
    }
    direction = BACKWARDS;
    move_in_direction(direction, DEFAULT_SPEED, ((sensor_values[direction] + sensor_values[direction + 1]) / 2.0 < GET_PARALLEL_TO_WALL) ? get_parallel(direction) : 0, veer_away_from_wall(direction));

    // Autonomous mode for the rest of the turns
    while (direction != DONE) {

        updateSensors();

        float avg_dist = (sensor_values[direction] + sensor_values[direction + 1]) / 2.0;
        
        // if the robot is close enough to turn  from a wall and its not at the second angled wall
        if (avg_dist < WALL_EPSILON + ((turns == BEFORE_RAMP) ? 2.54 : 0) && turns != SECOND_ANGLE) { // edit to account for angled wall and maybe ramp
            get_direction(direction);
            turns++;
        }
        // if the robot it close enough to the wall to turn and its at the turn with the second angled wall
        else if (avg_dist < WALL_EPSILON && turns == SECOND_ANGLE) {
            direction = RIGHT;
            turns++;
        }
        //printLCD((float) turns);

        uint8_t s = DEFAULT_SPEED;
        int8_t parallel = 0;

        float avg_dist_right = (sensor_values[(direction + 2) % 8] + sensor_values[((direction + 2) % 8) + 1]) / 2.0;
        float avg_dist_back = (sensor_values[(direction + 4) % 8] + sensor_values[((direction + 4) % 8) + 1]) / 2.0;

        // determine the speed at which the robot should move
        if (turns == RAMP && avg_dist_back < RAMP_BACK && avg_dist > RAMP_FRONT) {
            s = DEFAULT_SPEED - 50;
        }
        else if (turns == RAMP && avg_dist_right < RAMP_SIDE && avg_dist > RAMP_FRONT) {
            s = DEFAULT_SPEED;
        }
        else if (turns == BEFORE_RAMP) {
            s = DEFAULT_SPEED - 50;
        }
        else if (turns == SLOW) {
            s = DEFAULT_SPEED;// - 50;
        }

        // determine the wall, if any, the robot should get parallel to
        if (avg_dist < GET_PARALLEL_TO_WALL) {
            parallel = get_parallel(direction);
        }

        printLCD(avg_dist_right);
        // move in determined direction with speed s and get_parellel value parallel
        /*if (turns == RAMP && avg_dist_right < RAMP_SIDE && avg_dist > RAMP_FRONT) {
            drive_ramp(veer_away_from_wall(direction));
            last = !last;
            first = false;
        }
        else {*/
            move_in_direction(direction, s, parallel, (avg_dist < GET_PARALLEL_TO_WALL) ? -1 : veer_away_from_wall(direction));
        //}

        delay(10);
    }

    // stop robot
    frontL->run(RELEASE);
    frontR->run(RELEASE);
    rearL ->run(RELEASE);
    rearR ->run(RELEASE);
    delay(1000);

    printLCD("I'm done!!!!!!!");

    // Light dance at the end when the robot is done - for the lulz
    LED_1_OFF
    LED_2_OFF
    LED_3_OFF
    LED_4_OFF
    LED_5_OFF
    
    for (int i = 0; i < 10000; i++) {
        LED_1_ON
        delay(100);
        LED_1_OFF
        LED_2_ON
        delay(100);
        LED_2_OFF
        LED_3_ON
        delay(100);
        LED_3_OFF
        LED_4_ON
        delay(100);
        LED_4_OFF
        LED_5_ON
        delay(100);
        LED_5_OFF
        LED_4_ON
        delay(100);
        LED_4_OFF
        LED_3_ON
        delay(100);
        LED_3_OFF
        LED_2_ON
        delay(100);
        LED_2_OFF
    }
}

