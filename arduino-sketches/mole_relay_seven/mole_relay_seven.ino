#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);


int center = 300;//323;
int range = 250;//213;
int mi=center-range;//108;
int ma=center+range;//580;

int pos[]={center,center,center,center,center,center,center};
int pindex=0;

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
}

int inByte = 0;

void loop() {
  
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if (pindex<7) {
      if (inByte==1) pos[pindex++]=mi;
      if (inByte==2) pos[pindex++]=center; 
      if (inByte==3) pos[pindex++]=ma;
    }
    if (inByte==0 || inByte>=4)
    {
      if (pindex>=7) {
        int i=0;
        for (i=0; i<7; i++) pwm.setPWM(i, 0, pos[i]);
      }
      pindex=0;
    }
      
    delay(100); 
  }
}
