/*
 * Motorized curtains
 * That project is to build smart curtains device tha can
 * * Open curtains using button
 * * Oen curatins in the mosring using light sensor
 * * Open curtains using any IR remote ontrol
 * * Open curtains according to schedue
 * * control curtains from your phone
 * 
 * Project is build using cheap ULN2003 driver & stepper motor
 */
#include <Stepper.h>
#include <avr/sleep.h>
#include "LowPower.h"

void debug(String str){
  //Serial.println(str);
}

void debug(int num){
  //Serial.println(num);
}

const byte ledPin = 13;
const byte interruptPin = 2;
const int stepsPerRevolution = 2048;
const byte buttonPin = 5;

// Wiring:
// Pin 8 to IN1 on the ULN2003 driver
// Pin 9 to IN2 on the ULN2003 driver
// Pin 10 to IN3 on the ULN2003 driver
// Pin 11 to IN4 on the ULN2003 driver
// Create stepper object called 'stepper', note the pin order:
Stepper stepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

#define STOP 0
#define MOVE_UP   3 
#define MOVE_DOWN 4

#define DIRECTION 1
#define DEFAULT_MAX_POSITION 1000
#define DEFAULT_MIN_POSITION -1000

#define STEPS_PER_ITERATION 100
#define INTERRUPT_WHEN_HIGHT HIGH

volatile int state = STOP;
volatile int previousState = STOP;
int position = 0;
volatile int minPosition = -1000;
volatile int maxPosition = 1000;

static uint32_t wakeupTime=0;
static uint32_t buttonDebounceTime=0;

void setup() {  
  Serial.begin(9600);
  debug("brgin setup");
  stepper.setSpeed(6);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onButtonPressed, RISING );
}

void loop() {
  //debug(".");
  if(state == MOVE_DOWN){
    moveDown();
  } else if(state == MOVE_UP){    
    moveUp();
  } else {
    LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);
    delay(1000);
    wakeupUsbCharger();
  }
}

void onButtonPressed(){  
  if(millis() - buttonDebounceTime > 500){
    buttonDebounceTime = millis();
    debug("state=");
    debug(state);
    if(state != STOP){
      if(state == MOVE_DOWN && minPosition == DEFAULT_MIN_POSITION){
        minPosition = position;
      }else if (state == MOVE_DOWN && maxPosition == DEFAULT_MAX_POSITION){
        maxPosition = position;
      }
      stop();
    } else {
      changeMoveDirection();
    }
  }
}

void stop(){
  previousState = state;
  state = STOP;
  powerOffMotor();  
  debug("stop");  
}

void powerOffMotor(){
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  wakeupTime=millis();
}

void wakeupUsbCharger(){
  if(millis() - wakeupTime > 15000){
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
    delay(100);
    powerOffMotor();
  } 
}

void changeMoveDirection(){
  debug("changeMoveDirection");
  if(previousState == MOVE_DOWN){
    state = MOVE_UP;
  }else{
    state = MOVE_UP;
  }
}

void moveDown(){
  debug("moveDown");
  if(position < minPosition){
    stop();
    return;
  }
  stepper.step(STEPS_PER_ITERATION * DIRECTION);
  
  position--;
  debug(position);
}

void moveUp(){
  debug("moveUp");
  if(position >= maxPosition){
    stop();
    return;
  }
  stepper.step(-STEPS_PER_ITERATION * DIRECTION);
  
  position++;
  debug(position);
}
