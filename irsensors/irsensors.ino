// Arduino timer CTC interrupt example

// avr-libc library includes
#include <avr/power.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000

// set output pin for pulsing LED
#define LEDPIN 13

// compare value adjusted for CTC mode, 1/38000 sec = 420
#define timer_ticks 420

const byte numPins = 8;
byte digitalPins[] = {2,3,4,5,6,7,8,9};

int main(void)
{

// initialize timer
timer();

if (F_CPU == 16000000)
clock_prescale_set(clock_div_1);
  
// set sensor pins to detect objects
for(int i=2;i<10;i++)
{
 pinMode(i, INPUT);
}

// set output pin for pulsing LED
pinMode(LEDPIN, OUTPUT);

// enable global interrupts:
sei();

while(1);

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
digitalWrite(LEDPIN, !digitalRead(LEDPIN));
digitalWrite(LEDPIN, !digitalRead(LEDPIN));
}

// main loop
void loop() {
  
 byte num = Serial.read();
 
 for (byte i=0; i<numPins; i++) {
  byte state = bitRead(num, i);
  digitalWrite(digitalPins[i], state);
  Serial.print(state);
 }
 
  Serial.println();
}
