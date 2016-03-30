#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7); // RX, TX

void setup() {

  Serial.begin(9600);
  mySerial.begin(9600);

}

void loop()
{

  while (Serial.available()) {
      mySerial.write(Serial.read());
    }



  while (mySerial.available() > 0) {
    Serial.write(mySerial.read());
  }
  
}
