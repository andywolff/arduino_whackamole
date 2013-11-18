void setup() {
  Serial.begin(9600);
}

int inByte = 0;

void loop() {
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if (inByte=='H')
      Serial.print((char)(inByte+1));
    delay(100); 
  }
}
