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
#include <IRremote.h>

void debug(String str){
  Serial.println(str);
}

void debug(String str, int num){
  Serial.print(str);
  Serial.println(num);
}

const byte lightPin = A0;
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

#define STATE_STOPPED 0
#define STATE_MOVE      1
#define DIRECTION_UP   3 
#define DIRECTION_DOWN 4

#define DIRECTION 1
#define DEFAULT_MAX_POSITION 1000
#define DEFAULT_MIN_POSITION -1000

#define WAKEUP_INTERVAL 20000
#define BUTTON_DEBOUNCE_INTERVAL 500

#define STEPS_PER_ITERATION 100
#define INTERRUPT_WHEN_HIGHT HIGH

#define LIGHT_BARRIER 230
#define LIGHT_COUNTER_BARRIER 5

#define RECV_PIN 7

volatile int state = STATE_STOPPED;
volatile int direction = DIRECTION_DOWN;
int position = 0;
int lightLevel = 0;
int lightCounter = 0;
volatile int minPosition = -1000;
volatile int maxPosition = 1000;

static uint32_t wakeupTime=0;
static uint32_t buttonDebounceTime=0;

int irUp = 0;
int irDown = 0;

IRrecv irrecv(RECV_PIN);
decode_results results;

void setup() {  
  Serial.begin(9600);
  debug("begin setup");
  stepper.setSpeed(6);
  pinMode(lightPin, INPUT);
  irrecv.enableIRIn();
  irrecv.blink13(true);
  //pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), onButtonPressedInterruption, RISING );
}

void loop() { 
  //debug("state=", state);
  //delay(50);
  onButtonPressed();
  if(state == STATE_STOPPED){    
    //stop();
    delay(1000);
    wakeupTimer();
    //LowPower.idle(SLEEP_1S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);
  } else {    
    if(direction == DIRECTION_DOWN){
      moveDown();
    }else{
      moveUp();
    }
  }  
}

void onButtonPressedInterruption(){  
  if(millis() - buttonDebounceTime > BUTTON_DEBOUNCE_INTERVAL){
    buttonDebounceTime = millis();
    debug("state=", state);
    if(state == STATE_MOVE){
      learnMinMaxPosition();
      stop();
      //powerOffMotor();
    } else {
      changeMoveDirection();
    }    
  }
}

void onButtonPressed(){ 
  if (irrecv.decode(&results)){
    if(irUp == 0){
      irUp = results.value;
      debug("learn irUp=", irUp);
    } else if(irDown == 0){
      irDown = results.value;
      debug("learn irDown=", irDown);
    } else {
      if(state == STATE_MOVE){
        learnMinMaxPosition();
        stop();
      } else {
        if(results.value == irUp){
          direction = DIRECTION_UP;
        }else if(results.value == irDown) {
          direction = DIRECTION_DOWN;          
        }
        state = STATE_MOVE;
        debug("move by IR");
      }      
    }    
    //debug("results.value=", results.value);
    resetIdleTimer();
  }
  irrecv.resume();
}

void learnMinMaxPosition(){
    if(direction == DIRECTION_DOWN && minPosition == DEFAULT_MIN_POSITION){
      debug("learn minPosition=", position);
      minPosition = position;
    }else if (direction == DIRECTION_UP && maxPosition == DEFAULT_MAX_POSITION){
      debug("learn maxPosition=", position);
      maxPosition = position;
    }
  }

void stop(){
  debug("stop");
  state = STATE_STOPPED;
  powerOffMotor();
  resetIdleTimer();
}

void powerOffMotor(){
  debug("powerOffMotor");
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}

void wakeupTimer(){
  uint32_t diff= millis() - wakeupTime;
  //debug("try wakeupUsbCharger:", diff);
  if(diff > WAKEUP_INTERVAL){
    debug("wakeup");
    //wakeupUsbCharger();    
    checkLightLevel();
    powerOffMotor();
    resetIdleTimer();
  } 
}

void resetIdleTimer(){
  wakeupTime = millis();
}

void wakeupUsbCharger(){
  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  delay(100);
  powerOffMotor();
}

void checkLightLevel(){  
  int lightLevel = analogRead(lightPin);
  if(lightLevel > LIGHT_BARRIER){
    if(lightCounter < 10) lightCounter++;
  } else {
    if(lightCounter > 0) lightCounter--;
  }

  if((position <= minPosition + 5) && (lightCounter > LIGHT_COUNTER_BARRIER)){//curtains  closed
    state = STATE_MOVE;
    direction = DIRECTION_UP; // open curtain
    lightCounter = 10;
  } else if((position >= maxPosition - 5) && (lightCounter < LIGHT_COUNTER_BARRIER)){
    state = STATE_MOVE;
    direction = DIRECTION_DOWN; // open curtain
    lightCounter = 0;
  }

  //blink(lightLevel / 100);
  
  debug("lightLevel=", lightLevel);
  debug("lightCounter=", lightCounter);
  debug("position=", position);
  debug("minPosition=", minPosition);
  debug("maxPosition=", maxPosition);
}

void blink(int times){
  digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
    
  for(int i = 0 ; i<times; i++){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(180);
    digitalWrite(LED_BUILTIN, LOW);
    delay(180);
  }
}

void changeMoveDirection(){
  debug("changeMoveDirection");
  if(direction == DIRECTION_DOWN){
      direction = DIRECTION_UP;
  }else{
    direction = DIRECTION_DOWN;
  }
  state = STATE_MOVE;
}

void moveDown(){
  debug("moveDown");
  if(position <= minPosition){
    debug("minimum reached");
    stop();
    return;
  }
  stepper.step(STEPS_PER_ITERATION * DIRECTION);
  
  position--;
  debug("position=", position);
}

void moveUp(){
  debug("moveUp");
  if(position >= maxPosition){
    debug("maximum reached");
    stop();
    return;
  }
  stepper.step(-STEPS_PER_ITERATION * DIRECTION);
  
  position++;
  debug("position=", position);
}
