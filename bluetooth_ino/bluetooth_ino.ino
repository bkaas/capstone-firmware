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
    if( c == '$' ){
      for ( int i = 0; i<100; i++){
        mySerial.write("AT+RSSI?");
        delay(100);
        while (mySerial.available() > 0) {
          Serial.write(mySerial.read());
        }
      Serial.println();
      delay(100);
      }
    }  
    else {
      mySerial.write(c);
    }
    
  }



  while (mySerial.available() > 0) {
    Serial.write(mySerial.read());
    //Serial.println("goodbye");
  }
  
}
