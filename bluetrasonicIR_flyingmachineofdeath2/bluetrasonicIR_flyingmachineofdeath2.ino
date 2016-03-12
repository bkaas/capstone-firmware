
#include <PID_v1.h>
#include <SoftwareSerial.h>

#include <avr/power.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

#define timer_ticks 420 // compare value adjusted for CTC mode, 1/38000 sec = 420
#define LEDPIN 13 //pulsing LED
#define txrxPin A0
#define srfAddress2 0x01
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm 

//PID:
//Define Variables we'll be connecting to
double Input, Output;
double dist = 0;
double setPoint;
double p = 3, i=5, d=1;
//Specify the links and initial tuning parameters
PID myPID(&dist, &Output, &setPoint, p, i, d, DIRECT);
//



bool ultra = 0;

char blueval;

float thrLevel = 1201;
int rlLevel = 1500;
int ptchLevel = 1500;
int yawLevel = 1500;
uint8_t c;
String thr;
String tmpStr;
int trimStep = 2;
float Kp = 300.0;
int tmpInt;

int ultraMin = 1750;

// IR sensors
byte irPin1 = 10;
byte irPin2 = 11;
byte irPin3 = 12;
byte irPin4 = 13;

// Roll and Pitch movement stuff
//int sCount = 0; //stop counter
//int gCount = 0; //go counter
//int tmp;


SoftwareSerial UltrasonicBus(txrxPin, txrxPin);
SoftwareSerial blue(8, 7); //RX, TX

void setup() {
  
  setPoint = 40;
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(115200);
  while (!Serial && !blue.available());
  UltrasonicBus.begin(9600);
  while (!Serial && !UltrasonicBus.available());
  blue.begin(9600);

  // set sensor pins to detect objects
  pinMode(irPin1, INPUT);
  pinMode(irPin2, INPUT);
  pinMode(irPin3, INPUT);
  pinMode(irPin4, INPUT);

  // set output pin for pulsing LED
  pinMode(LEDPIN, OUTPUT);

}

void loop()  {

  if ( ultra ) {
    UltrasonicBus.listen();
    if (UltrasonicBus.isListening()) {
      dist = doRange(srfAddress2);
      thrLevel = thrPID(dist);

      tmpInt = int(thrLevel);
      Serial.print("t" + String(tmpInt));
    }
  }

  blue.listen();
  if (blue.isListening()) {
    delay(100); //does this really need to be this long? 50 sort of works?
    blueval = blue.read();
  }

  switch (blueval) {

    case 'a': //if(blueval == 'a'){  //ARM
      Serial.print(blueval);
      delay(20); //if we delay above, do we need delay here?
      thrLevel = 1201;
      break;

    case 'u': //if(blueval == 'u'){  //toggle ultrasonic measurements
      ultra ^= 1;
      break;

    case 't': //if (blueval == 't') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        thr += (char)c;
      }
      Serial.print("t" + thr);
      thrLevel = thr.toInt();
      thr = "";
      break;

    case 'w': //if(blueval == 'w'){   //down roll midVal (west)
      rlLevel -= trimStep;
      rlLevel = MAX(rlLevel, 1370);  //1370 trim min from multiwii
      Serial.print("r" + String(rlLevel));
      delay(20);
      break;

    case 'e': //if(blueval == 'e'){  //up roll midVal (east)
      rlLevel += trimStep;
      rlLevel = MIN(rlLevel, 1585);  //1585 trim max from multiwii
      Serial.print("r" + String(rlLevel));
      delay(20);
      break;

    case 's': //if(blueval == 's'){  //down pitch midVal (south)
      ptchLevel -= trimStep;
      ptchLevel = MAX(ptchLevel, 1370);
      Serial.print("p" + String(ptchLevel));
      delay(20);
      break;

    case 'n': //if(blueval == 'n'){  //up pitch midVal (north)
      ptchLevel += trimStep;
      ptchLevel = MIN(ptchLevel, 1585);
      Serial.print("p" + String(ptchLevel));
      delay(20);
      break;

    case 'l': //if(blueval == 'l'){  //down yaw midVal (left)
      yawLevel -= trimStep;
      yawLevel = MAX(yawLevel, 1370);
      Serial.print("y" + String(yawLevel));
      delay(20);
      break;

    case 'r': //if(blueval == 'r'){  //up yaw midVal (right)
      yawLevel += trimStep;
      yawLevel = MIN(yawLevel, 1585);
      Serial.print("y" + String(yawLevel));
      delay(20);
      break;

    case 'z': //if(blueval == 'z'){  //reset roll and pitch
      ptchLevel = 1500;
      rlLevel = 1500;
      yawLevel = 1500;
      Serial.print("p" + String(ptchLevel));
      delay(20);
      Serial.print("r" + String(rlLevel));
      delay(20);
      Serial.print("y" + String(yawLevel));
      delay(20);
      break;

    case 'p': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      p = double(tmpStr.toInt());
      tmpStr = "";
      break;

    case 'i': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      i = double(tmpStr.toInt());
      tmpStr = "";
      break;
      
      case 'd': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      d = double(tmpStr.toInt());
      tmpStr = "";
      break;
    case 'h': //height, changes setpoint
      for (int i = 0; i < 3; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      setPoint = tmpStr.toInt();
      tmpStr = "";
      break;

    
  }
  //    if(blueval == '!'){  //tell us what the roll and pitch is
  //      blue.write(rlLevel);
  //      delay(20);
  //      blue.write(ptchLevel);
  //      delay(20);
  //    }


  /******ROLL*******/
  //  if (digitalRead(digitalPin)==LOW){ //IR sensor sees something
  //    Serial.print("rg");  //roll go
  //    sCount = 0;
  //    gCount++;
  //    tmp = gCount;
  //  }
  //
  //  else if(digitalRead(digitalPin)==HIGH) {
  //    if(sCount < tmp) {
  //      Serial.print("rc");
  //    }
  //    else {
  //      Serial.print("rs"); //roll stop
  //    }
  //    sCount++;
  //    gCount = 0;
  //  }

}

float thrPID(int ultraDist) {
  float thrErr = 0;
  //myPID.SetTunings(3, 5, 2);
  myPID.Compute();
  thrLevel = 1800+0.784*Output;
  
  
  //thrErr = (setPoint - ultraDist);
  //thrLevel = thrLevel + thrErr * Kp / 100.0;
  
  //thrLevel = MIN(thrLevel, 1999);
  //thrLevel = MAX(thrLevel, ultraMin);

  return thrLevel;
}

int doRange(byte Address) {
  SRF01_Cmd(Address, getRange);                              // Calls a function to get range from SRF01
  while (UltrasonicBus.available() < 2);                     // Waits to get good data
  byte highByte = UltrasonicBus.read();                      // Get high byte
  byte lowByte = UltrasonicBus.read();                       // Get low byte
  int dist = ((highByte << 8) + lowByte);                    // Put them together
  return dist;
}

void SRF01_Cmd(byte Address, byte cmd) {                     // Function to send commands to the SRF01
  pinMode(txrxPin, OUTPUT);                                  // Set pin to output and send break by sending pin low, waiting 2ms and sending it high again for 1ms
  digitalWrite(txrxPin, LOW);
  delay(2);
  digitalWrite(txrxPin, HIGH);
  delay(1);
  UltrasonicBus.write(Address);                              // Send the address of the SRF01
  UltrasonicBus.write(cmd);                                  // Send commnd byte to SRF01
  pinMode(txrxPin, INPUT);                                   // Make input ready for Rx
  int availableJunk = UltrasonicBus.available();             // Filter out the junk data
  for (int x = 0; x < availableJunk; x++) {
    byte junk = UltrasonicBus.read();
  }
}
