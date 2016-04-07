#include <PID_v1.h>
#include <SoftwareSerial.h>

#include <avr/power.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

#define txrxPin A0
#define srfAddress2 0x02
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm 

//PID:
//Define Variables we'll be connecting to
double Input, Output;
double dist = 0;
double setPoint;
//Below setpoint constants:
double p = 2.8, i = 0.0, d = 0.0; //p=2.8, i=0.81

//Above setpoint constants:
double pa = 0.8*p, ia = 0.8*i, da = 0.0;


//Specify the links and initial tuning parameters
PID myPID(&dist, &Output, &setPoint, p, i, d, DIRECT);
//


char blueval;
bool ultra = 0;
bool infrared = 0;

float thrLevel = 1201;
int midVal[3] = {1591,1520,1518}; // {1491, 1522, 1561}; // pitch roll yaw midVal[1] = 1500; int midVal[0] = 1500; int midVal[2] = 1500;
int minThrottle = 1700;
int trimStep = 2;
int ptchLevel; int rlLevel;

uint8_t c;
String thr;
String tmpStr;
int tmpInt;

//Hermionie: N = 12, E = 11, S = 9, W = A5
//OldFuckful: N = 11, E = 9, S = A5, W = 12

// IR sensors
byte irPinN = 11; //12 ;
byte irPinE = 9; //11;
byte irPinS = A5; //9;
byte irPinW = 12; //A5;
byte enablePin = 13;

// Roll and Pitch movement stuff
bool state[4] = {1,1,1,1};  //north, east, south, west in that order
int count[4] = {0, 0, 0, 0};
int comp[4] = {0, 0, 0, 0};
int countPrev[4];
int tmpDir[4];
String dir[2] = {"p","r"};
int timeout=0, timeout1=0;
int direc, direc1;
unsigned long previousMillis = 0;
unsigned long previousMillis1, previousMillis2, previousMillis3, previousMillis4;
bool go = 1, go1 = 1;
int adjustment = 100;
int dip = 500;
double compFrac = 0.5;

SoftwareSerial UltrasonicBus(txrxPin, txrxPin);
SoftwareSerial blue(8, 7); //RX, TX

void setup() {

  setPoint = 100;
  myPID.SetMode(AUTOMATIC);

  Serial.begin(115200);
  while (!Serial && !blue.available());
  UltrasonicBus.begin(9600);
  while (!Serial && !UltrasonicBus.available());
  blue.begin(9600);

  // set sensor pins to detect objects
  pinMode(irPinN, INPUT);
  pinMode(irPinE, INPUT);
  pinMode(irPinS, INPUT);
  pinMode(irPinW, INPUT);

  pinMode(enablePin, OUTPUT);
  

}

void loop()  {
  unsigned long currentMillis = millis();
  
  if ( ultra ) {
    UltrasonicBus.listen();
    if (UltrasonicBus.isListening()) {

      dist = doRange(srfAddress2);
      thrLevel = thrPID();
      tmpInt = int(thrLevel);
      Serial.print("t" + String(tmpInt));
    }
  }
  
  if ( infrared && dist > 10 ) {
    
    /****PITCH/ROLL****/
    state[0] = digitalRead(irPinN);
    state[2] = digitalRead(irPinS);
    state[1] = digitalRead(irPinE);
    state[3] = digitalRead(irPinW);

    if (!state[0] && (currentMillis - previousMillis) >= 250) {
      Serial.print('x');
      previousMillis = currentMillis;
    }

    if (!state[2] && (currentMillis - previousMillis) >= 250) {
      Serial.print('y');
      previousMillis = currentMillis;
    }

    if (!state[1] && (currentMillis - previousMillis) >= 250) {
      Serial.print('z');
      previousMillis = currentMillis;
    }

    if (!state[3] && (currentMillis - previousMillis) >= 250) {
      Serial.print('u');
      previousMillis = currentMillis;
    }
    
  }

  blue.listen();
  if (blue.isListening()) {
    delay(50); //does this really need to be this long? 50 sort of works?
    blueval = blue.read();
  }

  switch (blueval) {

    case 'a': //if(blueval == 'a'){  //ARM
      Serial.print(blueval);
      //delay(20); //if we delay above, do we need delay here?
      thrLevel = 1201;    
      break;

    case 'u': //if(blueval == 'u'){  //toggle ultrasonic measurements, with ramp up to minimize initial error by Dan
      Serial.print('t'+ String(minThrottle+50));
      delay(30);
      
      ultra ^= 1;
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;

    case 'f':
      infrared ^= 1;
      digitalWrite(enablePin, infrared);
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
      midVal[1] -= trimStep;
      midVal[1] = MAX(midVal[1], 1370);  //1370 trim min from multiwii
      Serial.print("r" + String(midVal[1]));
      delay(20);
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;

    case 'e': //if(blueval == 'e'){  //up roll midVal (east)
      midVal[1] += trimStep;
      midVal[1] = MIN(midVal[1], 1585);  //1585 trim max from multiwii
      Serial.print("r" + String(midVal[1]));
      delay(20);
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;

    case 's': //if(blueval == 's'){  //down pitch midVal (south)
      midVal[0] -= trimStep;
      midVal[0] = MAX(midVal[0], 1370);
      Serial.print("p" + String(midVal[0]));
      delay(20);
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;

    case 'n': //if(blueval == 'n'){  //up pitch midVal (north)
      midVal[0] += trimStep;
      midVal[0] = MIN(midVal[0], 1585);
      Serial.print("p" + String(midVal[0]));
      delay(20);
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;

    case 'l': //if(blueval == 'l'){  //down yaw midVal (left)
      midVal[2] -= trimStep;
      midVal[2] = MAX(midVal[2], 1370);
      Serial.print("y" + String(midVal[2]));
      delay(20);
      break;

    case 'r': //if(blueval == 'r'){  //up yaw midVal (right)
      midVal[2] += trimStep;
      midVal[2] = MIN(midVal[2], 1585);
      Serial.print("y" + String(midVal[2]));
      delay(20);
      break;

    case 'z': //if(blueval == 'z'){  //reset roll and pitch
      midVal[0] = 1500;
      midVal[1] = 1500;
      midVal[2] = 1500;
      Serial.print("p" + String(midVal[0]));
      delay(20);
      Serial.print("r" + String(midVal[1]));
      delay(20);
      Serial.print("y" + String(midVal[2]));
      delay(20);
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;

    case 'p': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      p = double(tmpStr.toInt());
      p = p / 1000.0;
      tmpStr = "";
      break;

    case 'i': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      i = double(tmpStr.toInt());
      i = i / 1000.0;
      tmpStr = "";
      break;

    case 'd': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      d = double(tmpStr.toInt());
      d = d / 1000.0;
      tmpStr = "";
      break;

    case 'h': //height, changes setpoint
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      setPoint = tmpStr.toInt();
      tmpStr = "";
      break;

    case 'j': //(ad)justment
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      adjustment = tmpStr.toInt();
      tmpStr = "";
      break;

    case 'o': //
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      dip = tmpStr.toInt();
      tmpStr = "";
      break;

    case 'm': //minimum throttle
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      minThrottle = tmpStr.toInt();
      Serial.print("m" + tmpStr);
      tmpStr = "";
      delay(20);
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;
  
    case 'q': //EEPROM Clear
      Serial.print('e');
      delay(20); //if we delay above, do we need delay here?
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;

   case 'x': //fake pitch
      Serial.print('s');
      break;

   case 'g': //fraction of dip to compensate
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      compFrac = double(tmpStr.toInt());
      compFrac = compFrac / 1000.0;
      tmpStr = "";
      digitalWrite(enablePin, 1);
      delay(15);
      digitalWrite(enablePin, 0);
      break;
  }
}

float thrPID() {
  float thrErr = 0;
  myPID.Compute();
  thrLevel = minThrottle + map(Output, 0, 255, 0, 1999 - minThrottle);

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
