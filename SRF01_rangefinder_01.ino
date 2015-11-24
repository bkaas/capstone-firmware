#include <SoftwareSerial.h>
#define txrxPin 10                                           // Defines Pin 10 to be used as both rx and tx for the SRF01
#define srfAddress1 0x01                                     // Address of the SFR01
#define srfAddress2 0x02                                     // Address of the SFR01
#define srfAddress3 0x03                                     // Address of the SFR01
#define ledONE 7
#define ledTWO 6
#define ledTHREE 5
#define getSoft 0x5D                                         // Byte to tell SRF01 we wish to read software version
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm
#define getStatus 0x5F                                       // Byte used to get the status of the transducer

#define NumbSensors 2                                        // CHANGE AS NEEDED
#define NumbLED 2
int sensors[] = {srfAddress1,srfAddress2,srfAddress3};       // CHANGE AS NEEDED
int led[] = {ledONE, ledTWO, ledTHREE};

SoftwareSerial UltrasonicBus(txrxPin, txrxPin);              // Sets up the SoftwareSerial with Digital Pin 10 and sets the name to "UltrasonicBus"

void setup(){
  Serial.begin(19200);
  UltrasonicBus.begin(9600);                                    
  delay(200);                                                // Waits some time to make sure that SRF01 is powered up

  Serial.println("SRF01 Test");
  for (int i=0; i < NumbSensors; i++) {
    SensVers(i);                                                // Gets sensor versions and initializes sensors
  }
  Serial.println("Initialization Complete");
  for (int i=0; i< NumbLED; i++){
    digitalWrite(led[i],LOW);
  }

}

void loop(){
  Serial.print("Range avg =   ");
  float range[NumbSensors];
  for (int sensNumb = 0; sensNumb < NumbSensors; sensNumb++) {
   //checkLock(sensors[sensNumb]);                                // Calls a function to check if the transducer is locked or unlocked
   int max = 1;                                                 // Setup so that you can averaqe readings
   int sum = 0;
   
   for (int count = 0; count < max; count++) {
    sum = sum + doRange(sensors[sensNumb]);                     // Calls a function to get range from SRF01 and sums the ranges up so that you can take an avg
   }
   
   range[sensNumb] = sum/max; 
   Serial.print(range[sensNumb]);
   Serial.print("    ");

   if(range[sensNumb] >100 && range[sensNumb] <200){
    digitalWrite(led[sensNumb], HIGH);
   }
   else{
    digitalWrite(led[sensNumb], LOW);
   }

  }
  Serial.println("    ");                                        // Print range result to the screen               
  //changeAdress(0x01, 0x03);                                  // Changes address of sensor from (first address) to (seccond address)
}

void SensVers(int i){
    SRF01_Cmd(sensors[i], getSoft);                          // Calls a function to get the SRF01 software version
    while (UltrasonicBus.available() < 1);                   // Waits to get good data
    int softVer = UltrasonicBus.read();                      // Read software version from SRF01
    Serial.print("Sensor"); Serial.print(sensors[i]);
    Serial.print("Version:");                                       
    Serial.println(softVer);                                 // Prints the software version to LCD03
    delay(200);              
}


void changeAdress(byte oldA, byte newA){                       // Changes address of sensor from (first address) to (seccond address)
  SRF01_Cmd(oldA,0xA0);                                        // Preset commands to change the address. see data sheet
  SRF01_Cmd(oldA,0xAA);
  SRF01_Cmd(oldA,0xA5);
  SRF01_Cmd(oldA,newA);
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


void checkLock(byte Address) {
  SRF01_Cmd(Address, getStatus);                             // Call to a function that checks if the trancducer is locked or unlocked
  byte statusByte = UltrasonicBus.read();                    // Reads the SRF01 status, The least significant bit tells us if it is locked or unlocked
  int status = statusByte & 0x01;                            // Get status of lease significan bit
  if(status == 0) {                                      
    Serial.println("Unlocked");                              // Prints the word unlocked followd by a couple of spaces to make sure space after has nothing in
  }
   else {                                      
    Serial.println("Locked   ");                             // Prints the word locked followd by a couple of spaces to make sure that the space after has nothing in
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
