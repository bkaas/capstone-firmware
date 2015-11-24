// Arduino timer CTC interrupt example

// avr-libc library includes
#include <avr/power.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000

// set output pin for pulsing LED
#define LEDPIN 13

// compare value adjusted for CTC mode, 1/38000 sec = 209 using f = fcl/(2*N*(count + 1))
#define timer_ticks 420 //209

// ultrasonic definitions
#include <SoftwareSerial.h>

#define txrxPin 10                                           // Defines Pin 10 to be used as both rx and tx for the SRF01
#define srfAddress1 0x01                                     // Address of the SFR01
#define srfAddress2 0x02                                     // Address of the SFR01
#define srfAddress3 0x03                                     // Address of the SFR01
#define getSoft 0x5D                                         // Byte to tell SRF01 we wish to read software version
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm
#define getStatus 0x5F                                       // Byte used to get the status of the transducer

#define NumbSensors 1                                        // CHANGE AS NEEDED
int sensors[] = {srfAddress3};                               // CHANGE AS NEEDED

SoftwareSerial UltrasonicBus(txrxPin, txrxPin);              // Sets up the SoftwareSerial with Digital Pin 10 and sets the name to "UltrasonicBus"

void setup()
{
// set sensor pins to detect objects
for(int i=2;i<10;i++)
  {
   pinMode(i, INPUT);
  }

pinMode(LEDPIN, OUTPUT);

// ultrasonic setup
Serial.begin(19200);
UltrasonicBus.begin(9600);                                    
delay(200);                                                // Waits some time to make sure that SRF01 is powered up

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

// Set CS10 bit for no prescaler on Timer1:
TCCR1B |= (1 << CS10);
//TCCR1B |= (1 << CS12); // for clk16MHz/1024 prescaling

// On reset, cleared to 0
TCCR1C = 0;

// TCNT1 set timer counter initial value, clear to 0
TCNT1 = 0;

// enable timer compare interrupt:
TIMSK1 |= (1 << OCIE1A);

// enable global interrupts:
sei();
}

// flip LED pin HIGH/LOW on timer = comparison register value
ISR(TIMER1_COMPA_vect)
{
digitalWrite(LEDPIN, 0);
digitalWrite(LEDPIN, 1);
//TIFR1 |= (1 << OCF1A); // not sure if needed; this flag should automatically reset on ISR
}

const byte numPins = 8;
byte digitalPins[] = {2,3,4,5,6,7,8,9};

// main loop
void loop() {

 byte state = 0;

 for (byte i=0; i<numPins; i++) {
  byte state = digitalRead(digitalPins[i]);
  Serial.print(state);
 }
 
  Serial.println();

  delay(250);
  doRange(sensors[0]);
  Serial.print(doRange(sensors[0]));
  Serial.println("");  
}

void SRF01_Cmd(byte Address, byte cmd){                      // Function to send commands to the SRF01
  pinMode(txrxPin, OUTPUT);                                  // Set pin to output and send break by sending pin low, waiting 2ms and sending it high again for 1ms
  digitalWrite(txrxPin, LOW);                              
  delay(2);                                               
  digitalWrite(txrxPin, HIGH);                            
  delay(1);                                                
  UltrasonicBus.write(Address);                              // Send the address of the SRF01
  UltrasonicBus.write(cmd);                                  // Send commnd byte to SRF01
  pinMode(txrxPin, INPUT);                                   // Make input ready for Rx
  int availableJunk = UltrasonicBus.available();             // Filter out the junk data
  for(int x = 0; x < availableJunk; x++){
    byte junk = UltrasonicBus.read();
  }
}

int doRange(byte Address) {
  SRF01_Cmd(Address, getRange);                              // Calls a function to get range from SRF01
  while (UltrasonicBus.available() < 2);                     // Waits to get good data
  byte highByte = UltrasonicBus.read();                      // Get high byte
  byte lowByte = UltrasonicBus.read();                       // Get low byte
  int dist = ((highByte<<8)+lowByte);                        // Put them together
  return dist;
}
