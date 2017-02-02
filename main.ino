#include "global_variables.h"
#include "get_direction.h"
#include "move_direction.h"
#include "get_parallel.h"
#include "veer_away_from_wall.h"

#include "Wire.h"

#include "I2Cdev.h"
#include "MPU6050.h"

//accelerometer info
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

setep() {
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

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}

loop() {
    // as long as the robot is not done going through the maze
    while (direction != DONE) {
        // get data from gyroscope
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // set the current direction
        get_direction(direction);
        // move in that direction
        move_direction(direction, default_speed, get_parallel(direction), veer_away_from_wall(direction));
    }

    // stop robot
    move_direction(direction, 0, 0, -1);
}
