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

//Below setpoint constants:
double p = 2.8, i = 0.0, d = 0.0; //p=2.7, i=0.81

//Above setpoint constants:
double pa = 0.8*p, ia = 0.8*i, da = 0.0;


//Specify the links and initial tuning parameters
PID myPID(&dist, &Output, &setPoint, p, i, d, DIRECT);
//

//Variables of the major components
char blueval;
bool ultra = 0;
bool infrared = 0;
byte *packet;
bool goEvade = 0;

//
float thrLevel = 1201;
int midVal[3] = {1500, 1500, 1500}; // pitch roll yaw midVal[1] = 1500; int midVal[0] = 1500; int midVal[2] = 1500;
int minThrottle;
int trimStep = 2;
int ptchLevel; int rlLevel;

//Temporary values for data handling
uint8_t c;
String thr;
String tmpStr;
int tmpInt;
int tmpPitch, tmpRoll, tmpThrottle;

// IR sensors
byte irPinN = 12;
byte irPinE = 11;
byte irPinS = 9;
byte irPinW = A5;
byte enablePin = 13;

// Roll and Pitch movement stuff
int adjustment;
bool state[4];  //north, east, south, west in that order
int timeout=0, timeout1=0;
int direc, direc1;
unsigned long currentMillis;
unsigned long previousMillis1, previousMillis2, previousMillis3, previousMillis4;
bool go = 1, go1 = 1;
int dip;
int minDist;

SoftwareSerial UltrasonicBus(txrxPin, txrxPin);
SoftwareSerial blue(8, 7); //RX, TX

void setup() {
  myPID.SetMode(AUTOMATIC); // Initialize the PID

  loadSerials(); // Initialize serial communication
  loadDefaults(); // Load defaults
  initializePins(); // Initialize pinModes
}

void loop()  {
  currentMillis = millis();
  
  if ( ultra ) Serial.print("t" + String(runUltra()));
  if ( infrared && dist > minDist) {
    Serial.print("p" + String(runPitchIR()));
    Serial.print("r" + String(runRollIR()));
  }
  
  Serial.print("p" + String(runPitchIR()));
  state[0] = 0;
  runBlue();
}

/**************************************
 * All called functions are below. Five catagories:
 * 1) Send to PID black box.
 * 2) Communcation with Ultrasonic sensor.
 * 3) Setting pinModes and defaults for adjustable parameters.
 * 4) Main loop blocks (ultrasonic, IR, Bluetooth).
 */

// 1. PID black box.
float thrPID() {
  float thrErr = 0;
  myPID.Compute();
  thrLevel = minThrottle + map(Output, 0, 255, 0, 1999 - minThrottle);

  return thrLevel;
}

// 2. Communication with Ultrasonic sensor.
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

// 3. Setup for serial/software serials.
void loadSerials() {
  Serial.begin(115200);
  while (!Serial && !blue.available());
  UltrasonicBus.begin(9600);
  while (!Serial && !UltrasonicBus.available());
  blue.begin(9600);
}

// 3. Defaults for adjustable parameters.
void loadDefaults() {
  setPoint = 100; // Setpoint height in cm, adjust with 'h'
  minThrottle = 1740; // Minimum throttle on quad firmware, adjust with 'm'
  adjustment = 100; // Evasion strength, in pitch/roll increment, adjust with 'j'
  dip = 1000; // Time in millis before compensating, adjust with 'g'
  minDist = 40; // Minimum elevation to engage IR sensors, adjust with 'x'
}

// 3. pinMode setup for IR sensors.
void initializePins() {
  // set sensor pins to detect objects
  pinMode(irPinN, INPUT);
  pinMode(irPinE, INPUT);
  pinMode(irPinS, INPUT);
  pinMode(irPinW, INPUT);
  pinMode(enablePin, OUTPUT);
}

// 4. Ultrasonic main loop block.
int runUltra() {
    UltrasonicBus.listen();
    if (UltrasonicBus.isListening()) {

      dist = doRange(srfAddress2);
      thrLevel = thrPID();
      tmpInt = int(thrLevel);
    }
    return tmpInt;
}

// 4. IR Pitch main loop block.
int runPitchIR() {
  /****PITCH****/
  //state[0] = !digitalRead(irPinN); //fucked wit dis
  state[2] = 0; //!digitalRead(irPinS);
  
    if((state[0]-state[2]) && go){
      previousMillis1 = currentMillis;
      direc = adjustment*(state[2] - state[0]);
      timeout = 1;
      go = 0;
      return (midVal[0] + adjustment*(state[2] - state[0]));
    }
    else if (((currentMillis-previousMillis1) >= dip) && timeout==1 && !state[0] && !state[2]){
       previousMillis2 = currentMillis;
       timeout =2;
       go = 1;
       return (midVal[0] - (int)(0.75*direc));
     }
     else if (((currentMillis-previousMillis2) >= 500) && timeout==2){
       timeout = 0;
       return midVal[0];
     }
     else {
       goEvade = 0;
       return 0;
     }
}

// 4. IR Roll main loop block.
int runRollIR() {
    /****ROLL****/ 
  state[1] = 0;//!digitalRead(irPinE);
  state[3] = 0;//!digitalRead(irPinW); 
        
    if((state[1]-state[3]) && go1){
      previousMillis3 = currentMillis;
      direc1 = adjustment*(state[3] - state[1]);
      timeout1 = 1;
      go1 = 0;
      return (midVal[1] + adjustment*(state[3] - state[1]));
    }
    else if (((currentMillis-previousMillis3) >= dip) && timeout1==1 && !state[1] && !state[3]){
       previousMillis4 = currentMillis;
       timeout1 =2;
       go1 = 1;
       return (midVal[1] - (int)(0.75*direc1));
     }
     else if (((currentMillis-previousMillis4) >= 500) && timeout1==2){
       timeout1 = 0;
       return midVal[1];
     }
     else return 0;
}

// 4. Bluetooth main loop block.
void runBlue() {
  blue.listen();
  if (blue.isListening()) {
    delay(50); //does this really need to be this long? 50 sort of works?
    blueval = blue.read();
  }

  switch (blueval) {

    case 'a': //if(blueval == 'a'){  //ARM
      Serial.print(blueval);
      tmpThrottle = 1201;
      break;

    case 'u': //if(blueval == 'u'){  //toggle ultrasonic measurements, with ramp up to minimize initial error by Dan
      tmpThrottle = minThrottle + 50;   
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
      tmpThrottle = thr.toInt();
      thr = "";
      break;

    case 'w': //if(blueval == 'w'){   //down roll midVal (west)
      midVal[1] -= trimStep;
      midVal[1] = MAX(midVal[1], 1370);  //1370 trim min from multiwii
      Serial.print("r" + String(midVal[1]));
      delay(20);
      break;

    case 'e': //if(blueval == 'e'){  //up roll midVal (east)
      midVal[1] += trimStep;
      midVal[1] = MIN(midVal[1], 1585);  //1585 trim max from multiwii
      Serial.print("r" + String(midVal[1]));
      delay(20);
      break;

    case 's': //if(blueval == 's'){  //down pitch midVal (south)
      midVal[0] -= trimStep;
      midVal[0] = MAX(midVal[0], 1370);
      Serial.print("p" + String(midVal[0]));
      delay(20);
      break;

    case 'n': //if(blueval == 'n'){  //up pitch midVal (north)
      midVal[0] += trimStep;
      midVal[0] = MIN(midVal[0], 1585);
      Serial.print("p" + String(midVal[0]));
      delay(20);
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

    case 'x':   //set minimum distance to engage IRs
      for (int i = 0; i < 4; i++) {
        c = blue.read();
        tmpStr += (char)c;
      }
      minDist = tmpStr.toInt();
      tmpStr = "";
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
      break;
  
    case 'q': //EEPROM Clear
      Serial.print(blueval);
      delay(20); //if we delay above, do we need delay here?
      break;

    case '!': //EEPROM Clear
      state[0] = 1;
      break;
  }
}
