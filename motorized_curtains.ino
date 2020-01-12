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

const int ledPin = 13;
const int stepsPerRevolution = 2048;
const int buttonPin = 5;

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

int state = STOP;
int previousState = STOP;
int position = 0;
int minPosition = -1000;
int maxPosition = 1000;

void setup() {
  
  Serial.begin(9600);
  Serial.println("brgin setup");
  stepper.setSpeed(9);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  Serial.print(".");
  if(bttonPressed()){
    delay(1000);
    Serial.print("state=");
    Serial.println(state);
    if(state != STOP){
      if(state == MOVE_DOWN && minPosition == DEFAULT_MIN_POSITION){
        minPosition = position;
      }else if (state == MOVE_DOWN && maxPosition == DEFAULT_MAX_POSITION){
        maxPosition = position;
      }
      stop();
    } else {
      startMove();
    }
  }else{
    continueMove();
  }
}

void stop(){
  previousState = state;
  state = STOP;
  powerOffMotor();  
  Serial.println("stop");
}

void powerOffMotor(){
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}

void startMove(){
  Serial.println("startMove");
  if(previousState == MOVE_DOWN){
    moveUp();
  }else{
    moveDown();
  }
}

void continueMove(){  
  if(state == MOVE_DOWN){
    moveDown();
  } else if(state == MOVE_UP){    
    moveUp();
  }
}

void moveDown(){
  Serial.println("moveDown");
  if(position < minPosition){
    stop();
    return;
  }
  stepper.step(STEPS_PER_ITERATION * DIRECTION);
  state = MOVE_DOWN;
  position--;
  Serial.println(position);
}

void moveUp(){
  Serial.println("moveUp");
  if(position >= maxPosition){
    stop();
    return;
  }
  stepper.step(-STEPS_PER_ITERATION * DIRECTION);
  state = MOVE_UP;
  position++;
  Serial.println(position);
}

boolean bttonPressed(){
  int buttonState = digitalRead(buttonPin);
  return buttonState == 1;
}
