#include <PID_v1.h>
#include <SoftwareSerial.h>

#include <avr/power.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

#define txrxPin A0
#define srfAddress2 0x01
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm 

//PID:
//Define Variables we'll be connecting to
double Input, Output;
double dist = 0;
double setPoint;
double p = 4.0, i = 0.5, d = 0.0;
//Specify the links and initial tuning parameters
PID myPID(&dist, &Output, &setPoint, p, i, d, DIRECT);
//


char blueval;
bool ultra = 0;
bool infrared = 0;

float thrLevel = 1201;
int rlMid = 1500; int ptchMid = 1500; int yawMid = 1500;
int minThrottle = 1700;
int trimStep = 2;
int ptchLevel; int rlLevel;

uint8_t c;
String thr;
String tmpStr;
int tmpInt;

// IR sensors
byte irPinN = 11;
byte irPinE = 12;
byte irPinS = A5;
byte irPinW = 9;
byte enablePin = 13;

// Roll and Pitch movement stuff
int adjustment = 3;
bool state[4];  //north, east, south, west in that order
int count[4] = {0, 0, 0, 0};
int comp[4] = {0, 0, 0, 0};
int countPrev[4];
int tmpDir[4];


SoftwareSerial UltrasonicBus(txrxPin, txrxPin);
SoftwareSerial blue(8, 7); //RX, TX

void setup() {

  setPoint = 25;
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

  if ( ultra ) {
    UltrasonicBus.listen();
    if (UltrasonicBus.isListening()) {

      dist = doRange(srfAddress2);
      thrLevel = thrPID(dist);
      tmpInt = int(thrLevel);
      Serial.print("t" + String(tmpInt));
    }
  }

  if ( infrared ) {

    state[0] = digitalRead(irPinN);
    state[1] = digitalRead(irPinE);
    state[2] = digitalRead(irPinS);
    state[3] = digitalRead(irPinW);

    for (int i = 0; i < 4; i++) {
      countPrev[i] = count[i];
      count[i] = !state[i] * (count[i] + 1);
      if ((count[i] - countPrev[i]) < 0) tmpDir[i] = countPrev[i];
      if ( comp[i] < tmpDir[i] ) {
        comp[i] += 1;
      }
      else {
        comp[i] = 0;
        tmpDir[i] = 0;
      }
    }

    ptchLevel = ptchMid + ( count[0] - count[2] - (comp[0] - comp[2]) ) * adjustment;
    rlLevel = rlMid + ( count[1] - count[3] - (comp[1] - comp[3]) ) * adjustment;

    // Serial.print("p" + String(ptchLevel));
    // Serial.print("r" + String(rlLevel));
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
      rlMid -= trimStep;
      rlMid = MAX(rlMid, 1370);  //1370 trim min from multiwii
      Serial.print("r" + String(rlMid));
      delay(20);
      break;

    case 'e': //if(blueval == 'e'){  //up roll midVal (east)
      rlMid += trimStep;
      rlMid = MIN(rlMid, 1585);  //1585 trim max from multiwii
      Serial.print("r" + String(rlMid));
      delay(20);
      break;

    case 's': //if(blueval == 's'){  //down pitch midVal (south)
      ptchMid -= trimStep;
      ptchMid = MAX(ptchMid, 1370);
      Serial.print("p" + String(ptchMid));
      delay(20);
      break;

    case 'n': //if(blueval == 'n'){  //up pitch midVal (north)
      ptchMid += trimStep;
      ptchMid = MIN(ptchMid, 1585);
      Serial.print("p" + String(ptchMid));
      delay(20);
      break;

    case 'l': //if(blueval == 'l'){  //down yaw midVal (left)
      yawMid -= trimStep;
      yawMid = MAX(yawMid, 1370);
      Serial.print("y" + String(yawMid));
      delay(20);
      break;

    case 'r': //if(blueval == 'r'){  //up yaw midVal (right)
      yawMid += trimStep;
      yawMid = MIN(yawMid, 1585);
      Serial.print("y" + String(yawMid));
      delay(20);
      break;

    case 'z': //if(blueval == 'z'){  //reset roll and pitch
      ptchMid = 1500;
      rlMid = 1500;
      yawMid = 1500;
      Serial.print("p" + String(ptchMid));
      delay(20);
      Serial.print("r" + String(rlMid));
      delay(20);
      Serial.print("y" + String(yawMid));
      delay(20);
      break;

    case 'p': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      p = double(tmpStr.toInt());
      p = p / 100.0;
      tmpStr = "";
      break;

    case 'i': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      i = double(tmpStr.toInt());
      i = i / 100.0;
      tmpStr = "";
      break;

    case 'd': //if (blueval == 'k') {
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      d = double(tmpStr.toInt());
      d = d / 100.0;
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

    case 'm': //minimum throttle
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      minThrottle = tmpStr.toInt();
      tmpStr = "";
      break;
  
    case 'q': //EEPROM Clear
      Serial.print(blueval);
      delay(20); //if we delay above, do we need delay here?
      break;
  }
}

float thrPID(int ultraDist) {
  float thrErr = 0;
  myPID.Compute();
  //thrLevel = 1800+0.784*Output;
  thrLevel = minThrottle + map(Output, 0, 255, 0, 2050 - minThrottle);

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
