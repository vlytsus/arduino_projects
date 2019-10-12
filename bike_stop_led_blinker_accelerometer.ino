############################################
## Bicycle stop lights blinker.
############################################
## It blinks by left and rights led segments 
## if you ride with uniform speed.
## But if you brakes or stops it detects 
## acceleration and starts blinking faster 
## by all red led segments.
## That makes you more visible and
## indicates changes in your speed
##
## Following program is configured for 
## ADXL345 accelerometer
############################################

#include <Wire.h> // I2C library, gyroscope
#include <Adafruit_SleepyDog.h>

#define ACC (0xA7>>1)    //ADXL345 ACC address
#define A_TO_READ (6)    //num of bytes we are going to read each time (two bytes for each axis)


#define THRESHOLD_WARN  80
#define THRESHOLD_LIGHT 10

#define SLEEP_TIMER 10000

int blinker = 10;
int sleep_delay = 10000;

int acc[3];
int hx,hy,hz;
  
void initAcc() {
  //Turning on the ADXL345
  writeTo(ACC, 0x2D, 1<<3);      
  writeTo(ACC, 0x31, 0x0B);
  writeTo(ACC, 0x2C, 0x09);
  //by default the device is in +-2g range reading
}

void getAccelerometerData(int * result) {
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  byte buff[A_TO_READ];
  readFrom(ACC, regAddress, A_TO_READ, buff); //read the acceleration data from the ADXL345
  //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  //thus we are converting both bytes in to one int
  result[0] = (((int)buff[1]) << 8) | buff[0];   
  result[1] = (((int)buff[3])<< 8) | buff[2];
  result[2] = (((int)buff[5]) << 8) | buff[4];

}

void setup(){
  Serial.begin(9600);
  Wire.begin();
  initAcc();
  pinMode(13, OUTPUT); 
  pinMode(12, OUTPUT); 
}

void loop(){
  Serial.println(".");
  getAccelerometerData(acc);
  hx = acc[0];
  hy = acc[1];
  //hz = acc[2];
  
  if(hx > THRESHOLD_LIGHT || hy > THRESHOLD_LIGHT /*|| hz > THRESHOLD_LIGHT*/){
    sleep_delay = 10000; //reset sleep timer
  }
  
  if(hx > THRESHOLD_WARN){ 
   for(int i = 0; i < 8 ; i++){
    digitalWrite(13, HIGH);
    digitalWrite(12, HIGH);
    delay(90);
    digitalWrite(13, LOW);
    digitalWrite(12, LOW);
    delay(40);
   }
   sleep_delay = SLEEP_TIMER;
   
  }else{
  
    if(blinker > 0){
      blinker--;
      digitalWrite(13, HIGH);
      digitalWrite(12, LOW);
    }else if(blinker > -10){
      blinker--;
      digitalWrite(12, HIGH);
      digitalWrite(13, LOW);    
    }else{
      blinker = 10;
    }
      
    if(sleep_delay <= 0){
      digitalWrite(13, LOW);
      digitalWrite(12, LOW);
      Watchdog.sleep(1000);
      sleep_delay = 0;
      blinker = 10;
    }else{
      //do sleep & decrease sleep timer
      sleep_delay -= Watchdog.sleep(60);
    }    
  }
}


//---------------- Functions----------------------------------
//Writes val to address register on ACC
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on ACC in to buff array
void readFrom(int DEVICE, byte address, int num, byte buff[]) {
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from ACC
  int i = 0;
  while(Wire.available()){   //ACC may send less than requested (abnormal)
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
