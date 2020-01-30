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
//#include "LowPower.h"
#include <IRremote.h>

void debug(String str){
  Serial.println(str);
}

void debug(String str, int num){
  Serial.print(str);
  Serial.println(num);
}

//PINS SETUP --BEGIN---
#define PIN_IR_RECV 7
#define PIN_AMBIENT_LIGHT A0
#define PIN_BUTTON_INTERRUPT 2
#define PIN_HEIGHT_POTENTIOMETR A5
//PINS SETUP --END---

//STEPPER SETUP --BEGIN--
#define STEP_PER_REVOLUTION 2048
#define STEPS_PER_ITERATION 100
#define STEPPER_SPEED 5
Stepper stepper = Stepper(STEP_PER_REVOLUTION, 8, 10, 9, 11);
//STEPPER SETUP --END--

//IR RECEIVER SETUP --BEGIN--
IRrecv irrecv(PIN_IR_RECV);
decode_results irResults;
//IR RECEIVER SETUP --END--

//STATE SETUP --BEGIN--
#define STATE_STOPPED 0
#define STATE_MOVE      1
#define DIRECTION_UP   3 
#define DIRECTION_DOWN 4
#define DIRECTION 1
#define DEFAULT_MAX_POSITION 1000
#define DEFAULT_MIN_POSITION -1000
#define WAKEUP_INTERVAL 20000
#define LIGHT_BARRIER 230
#define LIGHT_COUNTER_BARRIER 5
#define BUTTON_DEBOUNCE_INTERVAL 500

int lightLevel = 0;
int lightCounter = 0;
int irBtnUpCode = 0; //Infrared control button up code
int irBtnDownCode = 0;
int windowHeight = 0; //window curtain height is configured by 10k potentiometer
static uint32_t buttonDebounceTime=0;
static uint32_t wakeupTime=0;

int minPosition = -1000; //volatile if power.h used
int maxPosition = 1000; //volatile if power.h used
int state = STATE_STOPPED; //volatile if power.h used
int direction = DIRECTION_DOWN; //volatile if power.h used
int position = 0;

//STATE SETUP --END--

void setup() {  
  Serial.begin(9600);
  //debug("begin setup");
  stepper.setSpeed(STEPPER_SPEED);
  pinMode(PIN_AMBIENT_LIGHT, INPUT);
  irrecv.enableIRIn();
  irrecv.blink13(true);
  //pinMode(PIN_BUTTON_INTERRUPT, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_INTERRUPT), onButtonPressedInterruption, RISING );
}

void loop() { 
  ////debug("state=", state);
  //delay(50);
  windowHeight = analogRead(PIN_HEIGHT_POTENTIOMETR); 
  //debug("windowHeight=", windowHeight); 
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

/*
void onButtonPressedInterruption(){  
  if(millis() - buttonDebounceTime > BUTTON_DEBOUNCE_INTERVAL){
    buttonDebounceTime = millis();
    //debug("state=", state);
    if(state == STATE_MOVE){
      learnMinMaxPosition();
      stop();
      //powerOffMotor();
    } else {
      changeMoveDirection();
    }    
  }
}
*/

void onButtonPressed(){ 
  if (irrecv.decode(&irResults)){
    int irResult = irResults.value;
    if(irBtnUpCode == 0){
      irBtnUpCode = irResult;      
      //debug("learn irBtnUpCode=", irBtnUpCode);
    } else if(irBtnDownCode == 0){
      irBtnDownCode = irResult;
      //debug("learn irBtnDownCode=", irBtnDownCode);
    } else {
      if(state == STATE_MOVE){
        learnMinMaxPosition();
        stop();
      } else {
        if(irResult == irBtnUpCode){
          direction = DIRECTION_UP;
          //debug("move UP by IR");
          //debug("irResults.value=", irResult);
          state = STATE_MOVE;
        }else if(irResult == irBtnDownCode) {
          direction = DIRECTION_DOWN;          
          //debug("move DOWN by IR");
          //debug("irResults.value=", irResult);
          state = STATE_MOVE;
        } else {
          //debug("test irBtnDownCode=", irBtnUpCode);
          //debug("test irBtnDownCode=", irBtnDownCode);          
          //debug("test irResults.value=", irResult);
        }
      }      
    }    
    resetIdleTimer();
  }
  irrecv.resume();
}

void learnMinMaxPosition(){
    /*if(direction == DIRECTION_DOWN && minPosition == DEFAULT_MIN_POSITION){
      //debug("learn minPosition=", position);
      minPosition = position;
    }else */    
    if (direction == DIRECTION_UP && maxPosition == DEFAULT_MAX_POSITION){
      //debug("learn maxPosition=", position);
      maxPosition = position;
      minPosition = maxPosition - windowHeight;
    }
  }

void stop(){
  //debug("stop");
  state = STATE_STOPPED;
  powerOffMotor();
  resetIdleTimer();
}

void powerOffMotor(){
  //debug("powerOffMotor");
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}

void wakeupTimer(){
  uint32_t diff = millis() - wakeupTime;
  if(diff > WAKEUP_INTERVAL){
    //debug("wakeup");
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
  int lightLevel = analogRead(PIN_AMBIENT_LIGHT);
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
  
  //debug("lightLevel=", lightLevel);
  //debug("lightCounter=", lightCounter);
  //debug("position=", position);
  //debug("minPosition=", minPosition);
  //debug("maxPosition=", maxPosition);
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
  //debug("changeMoveDirection");
  if(direction == DIRECTION_DOWN){
      direction = DIRECTION_UP;
  }else{
    direction = DIRECTION_DOWN;
  }
  state = STATE_MOVE;
}

void moveDown(){
  //debug("moveDown");
  if(position <= minPosition){
    //debug("minimum reached");
    stop();
    return;
  }
  stepper.step(STEPS_PER_ITERATION * DIRECTION);
  
  position--;
  //debug("position=", position);
}

void moveUp(){
  //debug("moveUp");
  if(position >= maxPosition){
    //debug("maximum reached");
    stop();
    return;
  }
  stepper.step(-STEPS_PER_ITERATION * DIRECTION);
  
  position++;
  //debug("position=", position);
}
