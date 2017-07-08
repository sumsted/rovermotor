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

Servo motor_left;
Servo motor_right;
byte incomingByte;
bool commandProcessed = false;

void run_motor(int speed, int orientation){
    int pulse = PWM_STOP;
    Serial.print("right: "+String(speed));
    if(speed > 100 || speed < -100){
        motor_right.write(PWM_STOP);
        Serial.println(", threshold pulse: "+String(PWM_STOP));
    } else {
        pulse = PWM_STOP + ((PWM_FULL_FORWARD - PWM_STOP) * (speed*PWM_TUNE_PERCENTAGE)/100 * orientation);
        motor_right.write(pulse);
        Serial.println(", pulse: "+String(pulse));
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

// ISR(TIMER1_OVF_vect) {
//     uint16_t ticks = micros();
//     if((ticks%SAFETY_CADENCE_MS) == 0){
//         if(!commandProcessed){
//             motor_left.write(PWM_STOP);
//             motor_right.write(PWM_STOP);
//             Serial.println("** stop **");
//         }
//         commandProcessed = false;
// //        Serial.println("ticks : "+String(ticks)+" cadence : "+String(SAFETY_CADENCE_MS));
//     }
//     Serial.println("ticks : "+String(ticks)+" cadence : "+String(SAFETY_CADENCE_MS));
// }

void safetyCheck() {
    if(!commandProcessed){
        motor_left.write(PWM_STOP);
        motor_right.write(PWM_STOP);
        Serial.println("commandProcessed is false");
        Serial.println("** safety stop **");
    } else {
        Serial.println("commandProcessed is true");
    }
    commandProcessed = false;
    Serial.println("safetyCheck");
}

SimpleTimer timer;

void setup() {
    Serial.begin(9600);
    while(!Serial){}
    Serial.println("begin");
    timer.setInterval(SAFETY_CADENCE_MS, safetyCheck);

    // noInterrupts();
    // TIMSK1 |= (1<<TOIE1);
    // interrupts();
    // safety timer
    // noInterrupts();           // disable all interrupts
    // TCCR1A = 0;
    // TCCR1B = 0;
    // TCNT1  = 0;

    // OCR1A = 31250;            // compare match register 16MHz/256/2Hz
    // TCCR1B |= (1 << WGM12);   // CTC mode
    // TCCR1B |= (1 << CS12);    // 256 prescaler 
    // TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    // interrupts(); 

    pinMode(LED_PIN, OUTPUT);
    motor_right.attach(MOTOR_PIN_RF);
}

void serialHandler(){
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        Serial.print(" I received:");
        Serial.println(incomingByte);
        switch(incomingByte){
            case 'w':
                move(BOT_FORWARD, 50);
                commandProcessed = true;
                break;
            case 's':
                move(BOT_BACKWARD, 50);
                commandProcessed = true;
                break;
            case 'a':
                move(BOT_ROTATE_LEFT, 50);
                commandProcessed = true;
                break;
            case 'd':
                move(BOT_ROTATE_RIGHT, 50);
                commandProcessed = true;
                break;
            case 'x':
                move(BOT_ROTATE_RIGHT, 50);
                commandProcessed = true;
                break;
            case ' ':
                motor_right.detach();
                commandProcessed = true;
                break;
        }
    }
}

void loop() {
    serialHandler();
    timer.run();
    // blink_led(100);
}
