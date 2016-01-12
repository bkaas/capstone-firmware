#include <SoftwareSerial.h>
#define txrxPin 2
#define srfAddress2 0x02
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm 
#define jump 10
#define setpoint 30
int c = 1000;

SoftwareSerial mySerial(10, 11); // RX, TX
SoftwareSerial UltrasonicBus(txrxPin, txrxPin);

void setup() {
//  Serial.begin(9600);
  while (!Serial);
  mySerial.begin(9600);
  while (!Serial);
  UltrasonicBus.begin(9600);
}

void loop()  {
  int dist = doRange(srfAddress2);
//  Serial.print(dist);
//  Serial.print("      ");
//  Serial.println(c);
//  delay(20);
  
  if(dist < setpoint &&  c < 1951){
    c = c + jump;
    mySerial.print(c);
    delay(20);
  }
  else if(dist > setpoint && c > 1000){
    c = c - jump;
    mySerial.print(c);
    delay(20);
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
