#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "EEPROM.h"
#include "Output.h"
#include "MultiWii.h"
#include "Serial.h"
#include "Protocol.h"
#include "RX.h"
#include <EEPROM.h>

#define MAX(a, b) ((a > b) ? a : b)
String thrLevel;
String rlLevel;
String ptchLevel;
String yawLevel;
String tmpString;

int minnierThrottle = 1650;
int tmpInt;
int midPitch = 1500;
int midRoll = 1515;

//********************Roll and Pitch***************//
bool state[4] = {0,0,0,0};  //north, east, south, west in that order
int timeout=0, timeout1=0;
int direc, direc1;
unsigned long previousMillis1, previousMillis2, previousMillis3, previousMillis4;
bool go = 1, go1 = 1;
int adjustment = 150;
int adjustmentP = 0;
int adjustmentR = 0;
int dip = 500;
double compFrac = 0.5;
bool infrared = 0;
int count = 0;
int count2 = 0;


static uint8_t CURRENTPORT = 0;

#define INBUF_SIZE 64
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];
static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdMSP[UART_NUMBER];

void evaluateOtherData(uint8_t sr);
#ifndef SUPPRESS_ALL_SERIAL_MSP
void evaluateCommand();
#endif
//
//#define BIND_CAPABLE 0;  //Used for Spektrum today; can be used in the future for any RX type that needs a bind and has a MultiWii module.
//#if defined(SPEK_BIND)
//  #define BIND_CAPABLE 1;
//#endif
//// Capability is bit flags; next defines should be 2, 4, 8...
//
//const uint32_t capability = 0+BIND_CAPABLE;
//

uint8_t read8()  {
  return inBuf[indRX[CURRENTPORT]++][CURRENTPORT] & 0xff;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
//
//void serialize8(uint8_t a) {
//  SerialSerialize(CURRENTPORT, a);
//  checksum[CURRENTPORT] ^= a;
//}
//void serialize16(int16_t a) {
//  serialize8((a   ) & 0xFF);
//  serialize8((a>>8) & 0xFF);
//}
//void serialize32(uint32_t a) {
//  serialize8((a    ) & 0xFF);
//  serialize8((a>> 8) & 0xFF);
//  serialize8((a>>16) & 0xFF);
//  serialize8((a>>24) & 0xFF);
//}
//
//void headSerialResponse(uint8_t err, uint8_t s) {
//  serialize8('$');
//  serialize8('M');
//  serialize8(err ? '!' : '>');  //if err=0 (false) set to >   - Kaas
//  checksum[CURRENTPORT] = 0; // start calculating a new checksum
//  serialize8(s);
//  serialize8(cmdMSP[CURRENTPORT]);
//}

//void headSerialReply(uint8_t s) {
//  headSerialResponse(0, s);
//}
//
//void inline headSerialError(uint8_t s) {
//  headSerialResponse(1, s);
//}
//
//void tailSerialReply() {
//  serialize8(checksum[CURRENTPORT]);UartSendData(CURRENTPORT);
//}
//
//void serializeNames(PGM_P s) {
//  headSerialReply(strlen_P(s));
//  for (PGM_P c = s; pgm_read_byte(c); c++) {
//    serialize8(pgm_read_byte(c));
//  }
//}
//
//Check incoming data if it matches the MSP protocol.
void serialCom() {
  //  CURRENTPORT = 0;
  //  uint8_t c,n;
  //  uint8_t cc = SerialAvailable(CURRENTPORT);
  //  while( cc-- ){
  //    uint8_t bytesTXBuff = SerialUsedTXBuff(CURRENTPORT); // indicates the number of occupied bytes in TX buffer
  //    if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return;  // ensure there is enough free TX buffer to go further (50 bytes margin)......TX_BUFFER_SIZE 128
  //    c = SerialRead(CURRENTPORT);
  //    if(c == 'a'){
  //      f.ARMED ^= 1;
  //      conf.throttleIn = 1350;
  //    }
  //  }
  unsigned long currentMillis = millis();
  uint8_t c, n;
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER];// = IDLE;

for (n = 0; n < UART_NUMBER; n++) {
#if !defined(PROMINI)
    CURRENTPORT = n;
#endif
#define GPS_COND
#if defined(GPS_SERIAL)
#if defined(GPS_PROMINI)
#define GPS_COND
#else
#undef GPS_COND
#define GPS_COND  && (GPS_SERIAL != CURRENTPORT)
#endif
#endif
#define SPEK_COND
#if defined(SPEKTRUM) && (UART_NUMBER > 1)
#define SPEK_COND && (SPEK_SERIAL_PORT != CURRENTPORT)
#endif
#define SBUS_COND
#if defined(SBUS) && (UART_NUMBER > 1)
#define SBUS_COND && (SBUS_SERIAL_PORT != CURRENTPORT)
#endif

    uint8_t cc = SerialAvailable(CURRENTPORT);

    while (cc-- GPS_COND SPEK_COND SBUS_COND) {
      uint8_t bytesTXBuff = SerialUsedTXBuff(CURRENTPORT); // indicates the number of occupied bytes in TX buffer
      if (bytesTXBuff > TX_BUFFER_SIZE - 50 ) return; // ensure there is enough free TX buffer to go further (50 bytes margin)
      c = SerialRead(CURRENTPORT);

      switch (c) {

        /****ARM****/
        case 'a':
          f.ARMED ^= 1;
          conf.throttleIn = 1100;
          break;

        case 'q':
          for (int i = 0 ; i < EEPROM.length() ; i++) {
            EEPROM.write(i, 0);
          }
          break;
        /*****SETVALS******/
        case 's':
            midPitch = rcData[PITCH];
            midRoll = rcData[ROLL];
          break;
        /*****THROTTLE*****/
        case 't':
          if (f.ARMED == 1) {
            for (int i = 0; i < 4; i++) {
              c = SerialRead(CURRENTPORT);
              thrLevel += (char)c;
            }
            tmpInt = MAX(minnierThrottle, thrLevel.toInt());
            conf.throttleIn = tmpInt;
            thrLevel = "";
          }
          break;

        /******ROLL*****/
        case 'r':
          if (f.ARMED == 1) {
            for (int i = 0; i < 4; i++) {
              c = SerialRead(CURRENTPORT);
              rlLevel += (char)c;
            }
            conf.rollIn = rlLevel.toInt();
            rlLevel = "";
          }
          break;

        /******PITCH*****/
        case 'p':
          if (f.ARMED == 1) {
            for (int i = 0; i < 4; i++) {
              c = SerialRead(CURRENTPORT);
              ptchLevel += (char)c;
            }
            conf.pitchIn = ptchLevel.toInt();
            ptchLevel = "";
          }
          break;

        /*******YAW*******/
//        case 'y':
//          if (f.ARMED == 1) {
//            for (int i = 0; i < 4; i++) {
//              c = SerialRead(CURRENTPORT);
//              ptchLevel += (char)c;
//            }
//            conf.yawIn = yawLevel.toInt();
//            yawLevel = "";
//          }
//          break;
        /**********MINNIERTHROTTLE***********/
        case 'm':
          if (f.ARMED == 1) {
            for (int i = 0; i < 4; i++) {
              c = SerialRead(CURRENTPORT);
              tmpString += (char)c;
            }
            minnierThrottle=tmpString.toInt();
            tmpString = "";
          }
          break;
        /************* PITCH/ROLL ****************/
        case 'x':
            state[0]=1;
            state[1]=0;
            state[2]=0;
            state[3]=0;
            adjustmentP = 0;
            count = 0;
            count2 = 0;
          break;

        case 'y':
            state[0]=0;
            state[1]=0;
            state[2]=1;
            state[3]=0;
            adjustmentP = 0;
            count = 0;
            count2 = 0;
          break;
        
        case 'z':
            state[0]=0;
            state[1]=1;
            state[2]=0;
            state[3]=0;
            adjustmentR = 0;
            count=0;
            count2 = 0;
          break;

        case 'u':
            state[0]=0;
            state[1]=0;
            state[2]=0;
            state[3]=1;
            adjustmentR = 0;
            count = 0;
            count2 = 0;
          break;
      }
    }
      
      /****PITCH/ROLL****/
        conf.pitchIn = midPitch - adjustment*(state[0] - state[2]) + adjustmentP;
        conf.rollIn = midRoll - adjustment*(state[1] - state[3]) + adjustmentR;
        conf.throttleIn = MAX((conf.throttleIn + 50*(state[0] || state[1] || state[2] || state[3])),1999);
        
        if(count > 120 && !state[0] && !state[1] && !state[2] && !state[3]){
          adjustmentP = adjustment*(state[0] - state[2]);
          adjustmentR = adjustment*(state[1] - state[3]);
          state[0] = 0;
          state[1] = 0;
          state[2] = 0;
          state[3] = 0;
          count = 0;
          count2 = 1;
        }
        
        if(count2 > 100) {
          adjustmentP = 0;
          adjustmentR = 0;
          count2 = 0;
        }
        
        count++;
        if (count2 > 0) count2++;
    

  }
}
