#include <SoftwareSerial.h>

SoftwareSerial mySerial(8, 7); // RX, TX

void setup() {

  Serial.begin(9600);
  while (!Serial);

  //pinMode(9,OUTPUT); digitalWrite(9,HIGH);

  Serial.println("Enter AT commands:");

  mySerial.begin(9600);
  //mySerial.println("hello?");

}

void loop()
{

//mySerial.print("broadcast");
//delay(2000);

  while (Serial.available()) {
    char c = Serial.read();
    mySerial.write(c);
    //Serial.write(c);
  }



  while (mySerial.available() > 0) {
    Serial.write(mySerial.read());
    //Serial.println("goodbye");
  }
  
}
