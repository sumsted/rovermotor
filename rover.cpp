#include <Arduino.h>
#include <Servo.h>
#include "SimpleTimer.h"

#define MOTOR_PIN_LF 3
#define MOTOR_PIN_RF 5

#define LED_PIN 13

#define MOTOR_ORIENTATION_LEFT 1
#define MOTOR_ORIENTATION_RIGHT -1

#define PWM_FULL_FORWARD 2000
#define PWM_STOP 1500
#define PWM_FULL_BACKWARD 1000
#define PWM_TUNE_PERCENTAGE .5

#define BOT_FORWARD 1
#define BOT_BACKWARD 2
#define BOT_ROTATE_LEFT 3
#define BOT_ROTATE_RIGHT 4
#define BOT_FORWARD_LEFT 5
#define BOT_FORWARD_RIGHT 6
#define BOT_BACKWARD_LEFT 7
#define BOT_BACKWARD_RIGHT 8
#define BOT_STOP 9

#define FAST 100
#define MEDIUM 50
#define SLOW 25
#define STOP 0

#define SAFETY_CADENCE_MS 500  // millisecs

#define MAX_BUFFER_SIZE 50

Servo motor_left;
Servo motor_right;
SimpleTimer timer;

bool commandProcessed = false; // check used by safety timer to tell if command issued
byte ledVal = HIGH; // safety timer flips the led on and off

void run_motor(int speed, int orientation){
    int pulse = PWM_STOP;
    if(speed > 100 || speed < -100){
        motor_right.write(PWM_STOP);
    } else {
        pulse = PWM_STOP + ((PWM_FULL_FORWARD - PWM_STOP) * (speed*PWM_TUNE_PERCENTAGE)/100 * orientation);
        motor_right.write(pulse);
    }
}

void run_motor_right(int speed){
    run_motor(speed, MOTOR_ORIENTATION_RIGHT);
}

void run_motor_left(int speed){
    run_motor(speed, MOTOR_ORIENTATION_LEFT);
}

void move(int direction, int speed)
{
   int leftSpeed = 0;
   int rightSpeed = 0;
   switch(direction){
     case BOT_FORWARD:
       leftSpeed = speed;
       rightSpeed = speed;
       break;        
     case BOT_BACKWARD:
       leftSpeed = -speed;
       rightSpeed = -speed;
       break;        
     case BOT_ROTATE_LEFT:
       leftSpeed = -speed;
       rightSpeed = speed;
       break;        
     case BOT_ROTATE_RIGHT:
       leftSpeed = speed;
       rightSpeed = -speed;
       break;        
     case BOT_FORWARD_LEFT:
       leftSpeed = speed/2;
       rightSpeed = speed;
       break;        
     case BOT_FORWARD_RIGHT:
       leftSpeed = speed;
       rightSpeed = speed/2;
       break;        
     case BOT_BACKWARD_LEFT:
       leftSpeed = -speed/2;
       rightSpeed = -speed;
       break;        
     case BOT_BACKWARD_RIGHT:
       leftSpeed = -speed;
       rightSpeed = -speed/2;
       break;        
     case BOT_STOP:
       leftSpeed = 0;
       rightSpeed = 0;
       break;        
     }
     run_motor_left(leftSpeed);
     run_motor_right(rightSpeed);
}

void  blink_led(byte ms){
    digitalWrite(LED_PIN, HIGH);
    delay(ms);
    digitalWrite(LED_PIN, LOW);
    delay(ms);
    digitalWrite(LED_PIN, HIGH);
    delay(ms);
    digitalWrite(LED_PIN, LOW);
    delay(ms*3);
}

void safetyCheck() {
    if(!commandProcessed){
        motor_left.write(PWM_STOP);
        motor_right.write(PWM_STOP);
    } else {
    }
    commandProcessed = false;
    ledVal = (ledVal == HIGH)?LOW:HIGH;
    digitalWrite(LED_PIN, ledVal);
}

void doStep(byte step){
    // for debugging
    // Serial.println("step: "+String(step));
    // delay(1000);
}

char *readEncoders(){
    return "{\"left\":321,\"right\":123}";
}

void serialHandler(){
    char readBuffer[MAX_BUFFER_SIZE] = "";
    String readString;
    doStep(1);

    while(Serial.available() > 0){ 
        // todo: may rewrite this to just read chars until end char
        // as i'd have to set time on readstring anyway 
        // and read until defaults to timeout if no end char
        // might be easier to understand if i just wrote it myself
        // for now all serial commands must end with a bang !
        // otherwise crap gets weird, slow read because of timeout
        // and safety kicks in immediately and shut everything down

        doStep(2);      
        readString = Serial.readStringUntil('!');
    }

    doStep(3);
    if(readString.length() > 0){
        doStep(4);
        readString.toCharArray(readBuffer, MAX_BUFFER_SIZE);
        // Serial.println("readBuffer: "+String(readBuffer));
        char command = readBuffer[0];
        int speed = atoi(readBuffer+1);
        // Serial.println("command: "+String(command)+", speedchars: "+String((readBuffer+1))+", speed: "+String(speed));
        switch(command){

            // general robot direction commands
            case 'w':
            case 'W':
                move(BOT_FORWARD, 50);
                commandProcessed = true;
                break;
            case 's':
            case 'S':
                move(BOT_BACKWARD, 50);
                commandProcessed = true;
                break;
            case 'a':
            case 'A':
                move(BOT_ROTATE_LEFT, 50);
                commandProcessed = true;
                break;
            case 'd':
            case 'D':
                move(BOT_ROTATE_RIGHT, 50);
                commandProcessed = true;
                break;
            case 'q':
            case 'Q':
                move(BOT_FORWARD_LEFT, 50);
                commandProcessed = true;
                break;
            case 'e':
            case 'E':
                move(BOT_FORWARD_RIGHT, 50);
                commandProcessed = true;
                break;
            case 'z':
            case 'Z':
                move(BOT_BACKWARD_LEFT, 50);
                commandProcessed = true;
                break;
            case 'c':
            case 'C':
                move(BOT_BACKWARD_RIGHT, 50);
                commandProcessed = true;
                break;

            // stop and detach, detach kills pulse to motors
            // while stop sends pwm 1500 which is middle for 
            // these frc controllers
            case 'x':
            case 'X':
                move(BOT_STOP, 0);
                commandProcessed = true;
                break;
            case ' ':
                motor_right.detach();
                commandProcessed = true;
                break;

            // control motors individually, just two motors for now
            // left motor y
            // right motor u
            // speed -100 to 100
            // format y-50   = left motor -50
            // positive speed is robot fwd and is calculated in run_motor...
            // and set using ORIENTATION macros
            case 'y':
            case 'Y':
                run_motor_left(speed);
                commandProcessed = true;
                break;
            case 'u':
            case 'U':
                run_motor_right(speed);
                commandProcessed = true;
                break;
        }
        Serial.write(readEncoders());
    }
}

void setup() {
    Serial.begin(9600);
    while(!Serial){}

    Serial.println("begin");
    timer.setInterval(SAFETY_CADENCE_MS, safetyCheck);
    pinMode(LED_PIN, OUTPUT);
    motor_right.attach(MOTOR_PIN_RF);
}

void loop() {
    serialHandler();
    timer.run();
}
