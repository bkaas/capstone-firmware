#include <SoftwareSerial.h>

#include <avr/power.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b) 

#define timer_ticks 420 // compare value adjusted for CTC mode, 1/38000 sec = 420
#define LEDPIN 13 //plusing LED
#define txrxPin 2
#define srfAddress2 0x03
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm 

bool ultra = 0;

int e = 2500;
char blueval;
int dist = 0;
int thrLevel;
int rlLevel = 1500;
int ptchLevel = 1500;
int yawLevel = 1500;
uint8_t c;
String thr;
int trimStep = 2;


// IR sensors
const byte numPins = 1;
byte digitalPin = 12;
int sCount = 0; //stop counter
int gCount = 0; //go counter
int tmp;

//SoftwareSerial quadSerial(10, 11); // RX, TX
SoftwareSerial UltrasonicBus(txrxPin, txrxPin);
SoftwareSerial blue(8,7); //RX, TX

void setup() {
  Serial.begin(115200);
//  while (!Serial);
//  quadSerial.begin(9600);
  while(!Serial && !blue.available());
  UltrasonicBus.begin(9600);
  while(!Serial && !UltrasonicBus.available());
  blue.begin(9600);

  // initialize timer
  timer();

  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  
  // set sensor pins to detect objects
  pinMode(digitalPin, INPUT);

  // set output pin for pulsing LED
  pinMode(LEDPIN, OUTPUT);

  // enable global interrupts:
  sei();
  //while(1);
  
}

void loop()  {

if( ultra ) {  
  UltrasonicBus.listen();
  if (UltrasonicBus.isListening()) {
    dist = doRange(srfAddress2);

    thrLevel = thrPID(dist);
    Serial.print("t" + String(thrLevel));
  }
}

  blue.listen();
  if (blue.isListening()) {
    delay(100); //does this really need to be this long? 50 sort of works?
    blueval = blue.read();
    
    if(blueval == 'a'){  //ARM
      Serial.print(blueval);
      delay(20); //if we delay above, do we need delay here?
    }
    if(blueval == 'u'){  //toggle ultrasonic measurements
      ultra ^= 1;
    }
    if (blueval == 't') {
        for(int i = 0; i < 4; i++){
          c = blue.read();
          thr += (char)c;
        }
        Serial.print("t" + thr);
        thr = "";
      }
    if(blueval == 'w'){   //down roll midVal (west)
      rlLevel -= trimStep;
      rlLevel = MAX(rlLevel, 1370);  //1370 trim min from multiwii
      Serial.print("r" + String(rlLevel));
      delay(20);
    }
    if(blueval == 'e'){  //up roll midVal (east)
      rlLevel += trimStep;
      rlLevel = MIN(rlLevel, 1585);  //1585 trim max from multiwii
      Serial.print("r" + String(rlLevel));
      delay(20);
    }
    if(blueval == 's'){  //down pitch midVal (south)
      ptchLevel -= trimStep;
      ptchLevel = MAX(ptchLevel, 1370);
      Serial.print("p" + String(ptchLevel));
      delay(20);
    }
    if(blueval == 'n'){  //up pitch midVal (north)
      ptchLevel += trimStep;
      ptchLevel = MIN(ptchLevel, 1585);
      Serial.print("p" + String(ptchLevel));
      delay(20);
    }
    if(blueval == 'l'){  //down yaw midVal (left)
      yawLevel -= trimStep;
      yawLevel = MAX(yawLevel, 1370);
      Serial.print("y" + String(yawLevel));
      delay(20);
    }
    if(blueval == 'r'){  //up yaw midVal (right)
      yawLevel += trimStep;
      yawLevel = MIN(yawLevel, 1585);
      Serial.print("y" + String(yawLevel));
      delay(20);
    }
    if(blueval == 'z'){  //reset roll and pitch
      ptchLevel = 1500;
      rlLevel = 1500;
      yawLevel = 1500;
      Serial.print("p" + String(ptchLevel));
      delay(20);
      Serial.print("r" + String(rlLevel));
      delay(20);
      Serial.print("y" + String(yawLevel));
      delay(20);
    }
//    if(blueval == '!'){  //tell us what the roll and pitch is
//      blue.write(rlLevel);
//      delay(20);
//      blue.write(ptchLevel);
//      delay(20);
//    }
  }

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

void timer()
{
// initialize Timer1
cli();          // disable global interrupts
TCCR1A = 0;     // set entire TCCR1A register to 0
TCCR1B = 0;     // same for TCCR1B

// set compare match register to desired timer count:
OCR1A = timer_ticks;
//OCR1AH = (timer_ticks >> 8);
//OCR1AL = (timer_ticks & 0x00FF);

// turn on CTC mode:
TCCR1B |= (1 << WGM12);

// Set CS10 bit for no prescaler:
TCCR1B |= (1 << CS10);

// On reset, cleared to 0
TCCR1C = 0;

// TCNT1 set timer counter initial value, clear to 0
TCNT1 = 0;

// enable timer compare interrupt:
TIMSK1 |= (1 << OCIE1A);

}

// flip LED pin HIGH/LOW
ISR(TIMER1_COMPA_vect)
{
  digitalWrite(LEDPIN, 1);
  digitalWrite(LEDPIN, 0);
}

int thrPID(int ultraDist) {
  int thrErr;
  int thrLevel;
  int setpoint = 50;

  thrErr = (setpoint - dist);
  thrLevel = 1600 + thrErr*4;  //deviate from float value
  thrLevel = MIN(thrLevel, 1999);
  thrLevel = MAX(thrLevel, 1201);

  return thrLevel;
}

