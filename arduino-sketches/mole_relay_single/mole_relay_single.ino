#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

/*
#define BUFFER_SIZE 100
char buffer[BUFFER_SIZE];
int bufferpos=0;

void clearbuffer() {
  int i=0;
  for (i=0; i<BUFFER_SIZE; i++) buffer[i]=0;
  bufferpos=0;
}
*/

int center = 300;//323;
int range = 250;//213;
int mi=center-range;//108;
int ma=center+range;//580;

int pos=center;

void setup() {
  Serial.begin(9600);
  
  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!

  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency
    
  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  TWBR = 12; // upgrade to 400KHz!
    
  //clearbuffer();
}

int inByte = 0;

void loop() {
  
  if (Serial.available() > 0) {
    inByte = Serial.read();
    
    if (inByte==0)
    {
      //int i;
      //for (i=0; i<bufferpos; i++) Serial.print((char)buffer[i]);
      pos=mi;
    } 
    if (inByte==1)
    {
      pos=center; 
    }
      
    if (inByte==2)
    {
     pos=ma; 
    }
    
    pwm.setPWM(0, 0, pos);
      
    delay(100); 
  }
}
