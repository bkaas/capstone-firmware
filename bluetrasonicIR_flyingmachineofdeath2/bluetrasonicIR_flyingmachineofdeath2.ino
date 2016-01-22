#include <SoftwareSerial.h>

#include <avr/power.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000

#define timer_ticks 420 // compare value adjusted for CTC mode, 1/38000 sec = 420
#define LEDPIN 13 //plusing LED
#define txrxPin 2
#define srfAddress2 0x02
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm 
#define jump 50
#define setpoint 30

int c = 1000;
int d = 0;
int e = 2500;
char blueval;

// IR sensors
const byte numPins = 1;
byte digitalPin = 5;

SoftwareSerial quadSerial(10, 11); // RX, TX
SoftwareSerial UltrasonicBus(txrxPin, txrxPin);
SoftwareSerial blue(8,7); //RX, TX

void setup() {
  pinMode(3, OUTPUT);
  Serial.begin(9600);
  while (!Serial);
  quadSerial.begin(9600);
  while(!Serial && !blue.available());
  UltrasonicBus.begin(9600);
  while(!Serial && !UltrasonicBus.available());
  blue.begin(9600);
  digitalWrite(3, HIGH);

  // initialize timer
  timer();

  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  
  // set sensor pins to detect objects
  pinMode(5, INPUT);

  // set output pin for pulsing LED
  pinMode(LEDPIN, OUTPUT);

  // enable global interrupts:
  sei();
  //while(1);
  
}

void loop()  {
  UltrasonicBus.listen();
  if (UltrasonicBus.isListening()) {
    int dist = doRange(srfAddress2);
    if (dist < setpoint &&  c < 1949) {
      c = c + jump;
        quadSerial.print(c); //swap with below for troubleshooting
  //    Serial.print(c);
      delay(20);
    }
    else if (dist > setpoint && c > 1000) {
      c = c - jump;
      quadSerial.print(c); //ditto
  //    Serial.print(c);
      delay(20);
    }
  }

  blue.listen();
  if (blue.isListening()) {
    delay(100); //does this really need to be this long? 50 sort of works?
    blueval = blue.read();
    if(blueval == 'a'){
//    Serial.print(blueval); //swap these two with below for troubleshooting
//    digitalWrite(3, !digitalRead(3));
      d = 9000;
      quadSerial.print(d);
      delay(20); //if we delay above, do we need delay here?
      d = 0;
      c = 0;
    }
  }

  if (digitalRead(digitalPin)==LOW){ //IR sensor sees something
    e = 2200; //set the roll to escape away from whatever it senses
    quadSerial.print(e);
    e = 2800; //counteract that roll to get back to stationary
    quadSerial.print(e);
    e = 2500; //hold stationary (MIDRC)
    quadSerial.print(e);
  }
  else {
    e = 2500; //if it doesn't sense something, hold stationary
    quadSerial.print(e);
  }
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
