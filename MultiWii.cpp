/*
  MultiWiiCopter by Alexandre Dubus
  www.multiwii.com
  November  2013     V2.3
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>
*/

#include <SoftwareSerial.h>

#include <avr/io.h>
#include "Wire.h"

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "EEPROM.h"
#include "IMU.h"
#include "Output.h"
#include "RX.h"
#include "Sensors.h"
#include "Serial.h"
#include "Protocol.h"

#include <avr/pgmspace.h>

int reader[4]; //added by Dan
int readIn; //added by Dan
String inString = ""; //added by Dan
String tmpstr;  //kaas
int tmpint;  //kaas

/***Ultrasonic variables***/
#define txrxPin 12                                           // Defines Pin 12 to be used as both rx and tx for the SRF01
#define srfAddress1 0x01
#define getRange 0x54                                        // Byte used to get range from SRF01 in cm
#define getStatus 0x5F
#define jump 50
#define setpoint 60
int ultraCount = 0;
int distAVG = 0;
int thrErr = 0;

/***Control Variables***/
int throttleErr = 1000; int rollErr; int pitchErr; int yawErr;
unsigned long previousMillis = 0;
const long interval = 150;

SoftwareSerial UltrasonicBus(txrxPin, txrxPin);

/****Control Variables****/  //added by Kaas
int counter = 0;
int rollZero; int pitchZero; int yawZero;
int rollVal; int pitchVal; int yawVal;


/*********** RC alias *****************/

const char pidnames[] PROGMEM =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
  ;

const char boxnames[] PROGMEM = // names for dynamic generation of config GUI
  "ARM;"
#if ACC
  "ANGLE;"
  "HORIZON;"
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  "BARO;"
#endif
#ifdef VARIOMETER
  "VARIO;"
#endif

#ifdef HEADFREE
  "HEADFREE;" // 5
#endif
#ifdef HEADHOLD
  "HEADHOLD;" // 6
#endif

#if MAG
  "MAG;"
  "HEADFREE;"
  "HEADADJ;"
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
  "CAMSTAB;"
#endif
#if defined(CAMTRIG)
  "CAMTRIG;"
#endif
#if GPS
  "GPS HOME;"
  "GPS HOLD;"
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  "PASSTHRU;"
#endif
#if defined(BUZZER)
  "BEEPER;"
#endif
#if defined(LED_FLASHER)
  "LEDMAX;"
  "LEDLOW;"
#endif
#if defined(LANDING_LIGHTS_DDR)
  "LLIGHTS;"
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
  "CALIB;"
#endif
#ifdef PID_SWITCH
  "PID SW;"
#endif
#ifdef OSD_SWITCH
  "OSD SW;"
#endif
  ;

const uint8_t boxids[] PROGMEM = {// permanent IDs associated to boxes. This way, you can rely on an ID number to identify a BOX function.
  0, //"ARM;"
#if ACC
  1, //"ANGLE;"
  2, //"HORIZON;"
#endif
#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
  3, //"BARO;"
#endif
#ifdef VARIOMETER
  4, //"VARIO;"
#endif
#if MAG
  5, //"MAG;"
  6, //"HEADFREE;"
  7, //"HEADADJ;"
#endif
#if defined(SERVO_TILT) || defined(GIMBAL)|| defined(SERVO_MIX_TILT)
  8, //"CAMSTAB;"
#endif
#if defined(CAMTRIG)
  9, //"CAMTRIG;"
#endif
#if GPS
  10, //"GPS HOME;"
  11, //"GPS HOLD;"
#endif
#if defined(FIXEDWING) || defined(HELICOPTER)
  12, //"PASSTHRU;"
#endif
#if defined(BUZZER)
  13, //"BEEPER;"
#endif
#if defined(LED_FLASHER)
  14, //"LEDMAX;"
  15, //"LEDLOW;"
#endif
#if defined(LANDING_LIGHTS_DDR)
  16, //"LLIGHTS;"
#endif
#ifdef INFLIGHT_ACC_CALIBRATION
  17, //"CALIB;"
#endif
#ifdef PID_SWITCH
  18, //"pid_sw;"
#endif
#ifdef OSD_SWITCH
  19, //"OSD_SWITCH;"
#endif
};


uint32_t currentTime = 0;
uint16_t previousTime = 0;
uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
uint16_t calibratingG;
int16_t  magHold, headFreeModeHold; // [-180;+180]
uint8_t  vbatMin = VBATNOMINAL;  // lowest battery voltage in 0.1V steps
uint8_t  rcOptions[CHECKBOXITEMS];
int32_t  AltHold; // in cm
int16_t  sonarAlt;
int16_t  BaroPID = 0;
int16_t  errorAltitudeI = 0;

// **************
// gyro+acc IMU
// **************
int16_t gyroZero[3] = {0, 0, 0};

imu_t imu;

analog_t analog;

alt_t alt;

att_t att;

#if defined(ARMEDTIMEWARNING)
uint32_t  ArmedTimeWarningMicroSeconds = 0;
#endif

int16_t  debug[4];

flags_struct_t f;

//for log
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY)
uint16_t cycleTimeMax = 0;       // highest ever cycle timen
uint16_t cycleTimeMin = 65535;   // lowest ever cycle timen
int32_t  BAROaltMax;             // maximum value
uint16_t GPS_speedMax = 0;       // maximum speed from gps
uint16_t powerValueMaxMAH = 0;
#endif
#if defined(LOG_VALUES) || defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
uint32_t armedTime = 0;
#endif

int16_t  i2c_errors_count = 0;
int16_t  annex650_overrun_count = 0;



#if defined(THROTTLE_ANGLE_CORRECTION)
int16_t throttleAngleCorrection = 0;	// correction of throttle in lateral wind,
int8_t  cosZ = 100;					// cos(angleZ)*100
#endif



// **********************
//Automatic ACC Offset Calibration
// **********************
#if defined(INFLIGHT_ACC_CALIBRATION)
uint16_t InflightcalibratingA = 0;
int16_t AccInflightCalibrationArmed;
uint16_t AccInflightCalibrationMeasurementDone = 0;
uint16_t AccInflightCalibrationSavetoEEProm = 0;
uint16_t AccInflightCalibrationActive = 0;
#endif

// **********************
// power meter
// **********************
#if defined(POWERMETER) || ( defined(LOG_VALUES) && (LOG_VALUES >= 3) )
uint32_t pMeter[PMOTOR_SUM + 1];  // we use [0:7] for eight motors,one extra for sum
uint8_t pMeterV;                  // dummy to satisfy the paramStruct logic in ConfigurationLoop()
uint32_t pAlarm;                  // we scale the eeprom value from [0:255] to this value we can directly compare to the sum in pMeter[6]
uint16_t powerValue = 0;          // last known current
#endif
uint16_t intPowerTrigger1;

// **********************
// telemetry
// **********************
#if defined(LCD_TELEMETRY)
uint8_t telemetry = 0;
uint8_t telemetry_auto = 0;
#endif
#ifdef LCD_TELEMETRY_STEP
char telemetryStepSequence []  = LCD_TELEMETRY_STEP;
uint8_t telemetryStepIndex = 0;
#endif

// ******************
// rc functions
// ******************
#define ROL_LO  (1<<(2*ROLL))
#define ROL_CE  (3<<(2*ROLL))
#define ROL_HI  (2<<(2*ROLL))
#define PIT_LO  (1<<(2*PITCH))
#define PIT_CE  (3<<(2*PITCH))
#define PIT_HI  (2<<(2*PITCH))
#define YAW_LO  (1<<(2*YAW))
#define YAW_CE  (3<<(2*YAW))
#define YAW_HI  (2<<(2*YAW))
#define THR_LO  (1<<(2*THROTTLE))
#define THR_CE  (3<<(2*THROTTLE))
#define THR_HI  (2<<(2*THROTTLE))

int16_t failsafeEvents = 0;
volatile int16_t failsafeCnt = 0;

int16_t rcData[RC_CHANS];    // interval [1000;2000]
int16_t rcSerial[8];         // interval [1000;2000] - is rcData coming from MSP
int16_t rcCommand[4];        // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
uint8_t rcSerialCount = 0;   // a counter to select legacy RX when there is no more MSP rc serial data
int16_t lookupPitchRollRC[5];// lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[11];// lookup table for expo & mid THROTTLE

#if defined(SPEKTRUM)
volatile uint8_t  spekFrameFlags;
volatile uint32_t spekTimeLast;
#endif

// *************************
// motor and servo functions
// *************************
int16_t axisPID[3];
int16_t motor[8];
int16_t servo[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1000};

// ************************
// EEPROM Layout definition
// ************************
static uint8_t dynP8[2], dynD8[2];

global_conf_t global_conf;

conf_t conf;

#ifdef LOG_PERMANENT
plog_t plog;
#endif

// **********************
// GPS common variables
// **********************
int16_t  GPS_angle[2] = { 0, 0};                      // the angles that must be applied for GPS correction
int32_t  GPS_coord[2];
int32_t  GPS_home[2];
int32_t  GPS_hold[2];
uint8_t  GPS_numSat;
uint16_t GPS_distanceToHome;                          // distance to home  - unit: meter
int16_t  GPS_directionToHome;                         // direction to home - unit: degree
uint16_t GPS_altitude;                                // GPS altitude      - unit: meter
uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
uint8_t  GPS_update = 0;                              // a binary toogle to distinct a GPS position update
uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
uint8_t  GPS_Enable  = 0;

// The desired bank towards North (Positive) or South (Negative) : latitude
// The desired bank towards East (Positive) or West (Negative)   : longitude
int16_t  nav[2];
int16_t  nav_rated[2];    //Adding a rate controller to the navigation to make it smoother

uint8_t nav_mode = NAV_MODE_NONE; // Navigation mode

uint8_t alarmArray[16];           // array

#if BARO
int32_t baroPressure;
int32_t baroTemperature;
int32_t baroPressureSum;
#endif

void annexCode() { // this code is excetuted at each loop and won't interfere with control loop if it lasts less than 650 microseconds
  static uint32_t calibratedAccTime;
  uint16_t tmp, tmp2;
  uint8_t axis, prop1, prop2;

  // PITCH & ROLL only dynamic PID adjustemnt,  depending on throttle value
  prop2 = 128; // prop2 was 100, is 128 now
  if (rcData[THROTTLE] > 1500) { // breakpoint is fix: 1500
    if (rcData[THROTTLE] < 2000) {
      prop2 -=  ((uint16_t)conf.dynThrPID * (rcData[THROTTLE] - 1500) >> 9); //  /512 instead of /500
    } else {
      prop2 -=  conf.dynThrPID;
    }
  }

  for (axis = 0; axis < 3; axis++) {
    tmp = min(abs(rcData[axis] - MIDRC), 500);
#if defined(DEADBAND)
    if (tmp > DEADBAND) {
      tmp -= DEADBAND;
    }
    else {
      tmp = 0;
    }
#endif
    if (axis != 2) { //ROLL & PITCH
      tmp2 = tmp >> 7; // 500/128 = 3.9  => range [0;3]
      rcCommand[axis] = lookupPitchRollRC[tmp2] + ((tmp - (tmp2 << 7)) * (lookupPitchRollRC[tmp2 + 1] - lookupPitchRollRC[tmp2]) >> 7);
      prop1 = 128 - ((uint16_t)conf.rollPitchRate * tmp >> 9); // prop1 was 100, is 128 now -- and /512 instead of /500
      prop1 = (uint16_t)prop1 * prop2 >> 7; // prop1: max is 128   prop2: max is 128   result prop1: max is 128
      dynP8[axis] = (uint16_t)conf.pid[axis].P8 * prop1 >> 7; // was /100, is /128 now
      dynD8[axis] = (uint16_t)conf.pid[axis].D8 * prop1 >> 7; // was /100, is /128 now
    } else {      // YAW
      rcCommand[axis] = tmp;
    }
    if (rcData[axis] < MIDRC) rcCommand[axis] = -rcCommand[axis];
  }
  tmp = constrain(rcData[THROTTLE], MINCHECK, 2000);
  tmp = (uint32_t)(tmp - MINCHECK) * 2559 / (2000 - MINCHECK); // [MINCHECK;2000] -> [0;2559]
  tmp2 = tmp / 256; // range [0;9]
  rcCommand[THROTTLE] = lookupThrottleRC[tmp2] + (tmp - tmp2 * 256) * (lookupThrottleRC[tmp2 + 1] - lookupThrottleRC[tmp2]) / 256; // [0;2559] -> expo -> [conf.minthrottle;MAXTHROTTLE]

  if (f.HEADFREE_MODE) { //to optimize
#if defined(HEADFREE) || defined(HEADHOLD)
    float radDiff = (att.heading - headFreeModeHold) * 0.0174533f;
#else
    float radDiff = (att.heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
#endif
    float cosDiff = cos(radDiff);
    float sinDiff = sin(radDiff);
    int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
    rcCommand[ROLL] =  rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
    rcCommand[PITCH] = rcCommand_PITCH;
  }

  // query at most one multiplexed analog channel per MWii cycle
  static uint8_t analogReader = 0;
  switch (analogReader++ % 3) {
#if defined(POWERMETER_HARD)
    case 0:
      {
        uint16_t pMeterRaw; // used for current reading
        static uint32_t lastRead = currentTime;
        static uint8_t ind = 0;
        static uint16_t pvec[PSENSOR_SMOOTH], psum;
        uint16_t p =  analogRead(PSENSORPIN);
        //LCDprintInt16(p); LCDcrlf();
        //debug[0] = p;
#if PSENSOR_SMOOTH != 1
        psum += p;
        psum -= pvec[ind];
        pvec[ind++] = p;
        ind %= PSENSOR_SMOOTH;
        p = psum / PSENSOR_SMOOTH;
#endif
        powerValue = ( conf.psensornull > p ? conf.psensornull - p : p - conf.psensornull); // do not use abs(), it would induce implicit cast to uint and overrun
        analog.amperage = powerValue * conf.pint2ma;
        pMeter[PMOTOR_SUM] += ((currentTime - lastRead) * (uint32_t)((uint32_t)powerValue * conf.pint2ma)) / 100000; // [10 mA * msec]
        lastRead = currentTime;
        break;
      }
#endif // POWERMETER_HARD

#if defined(VBAT)
    case 1:
      {
        static uint8_t ind = 0;
        static uint16_t vvec[VBAT_SMOOTH], vsum;
        uint16_t v = analogRead(V_BATPIN);
        //debug[1] = v;
#if VBAT_SMOOTH == 1
        analog.vbat = (v << 4) / conf.vbatscale; // result is Vbatt in 0.1V steps
#else
        vsum += v;
        vsum -= vvec[ind];
        vvec[ind++] = v;
        ind %= VBAT_SMOOTH;
#if VBAT_SMOOTH == 16
        analog.vbat = vsum / conf.vbatscale; // result is Vbatt in 0.1V steps
#elif VBAT_SMOOTH < 16
        analog.vbat = (vsum * (16 / VBAT_SMOOTH)) / conf.vbatscale; // result is Vbatt in 0.1V steps
#else
        analog.vbat = ((vsum / VBAT_SMOOTH) * 16) / conf.vbatscale; // result is Vbatt in 0.1V steps
#endif
#endif
        break;
      }
#endif // VBAT
#if defined(RX_RSSI)
    case 2:
      {
        static uint8_t ind = 0;
        static uint16_t rvec[RSSI_SMOOTH], rsum;
        uint16_t r = analogRead(RX_RSSI_PIN);
#if RSSI_SMOOTH == 1
        analog.rssi = r;
#else
        rsum += r;
        rsum -= rvec[ind];
        rvec[ind++] = r;
        ind %= RSSI_SMOOTH;
        r = rsum / RSSI_SMOOTH;
        analog.rssi = r;
#endif
        break;
      }
#endif
  } // end of switch()

  if ( (calibratingA > 0 && ACC ) || (calibratingG > 0) ) { // Calibration phasis
    LEDPIN_TOGGLE;
  } else {
    if (f.ACC_CALIBRATED) {
      LEDPIN_OFF;
    }
    if (f.ARMED)
    {
#if defined(VBAT)
      if ((analog.vbat > conf.vbatlevel_warn1)) {
        LEDPIN_ON;
      }
      else {
        static unsigned long lblink = millis() + 333;
        if (millis() >= lblink) {
          LEDPIN_TOGGLE;
          lblink = millis() + 333;
        }
      }
#else
      LEDPIN_ON;
#endif
    }
  }

#if defined(LED_RING)
  static uint32_t LEDTime;
  if ( currentTime > LEDTime ) {
    LEDTime = currentTime + 50000;
    i2CLedRingState();
  }
#endif

#if defined(LED_FLASHER)
  auto_switch_led_flasher();
#endif

  if ( currentTime > calibratedAccTime ) {
    if (! f.SMALL_ANGLES_25) {
      // the multi uses ACC and is not calibrated or is too much inclinated
      f.ACC_CALIBRATED = 0;
      LEDPIN_TOGGLE;
      calibratedAccTime = currentTime + 100000;
    } else {
      f.ACC_CALIBRATED = 1;
    }
  }

#if !(defined(SPEKTRUM) && defined(PROMINI))  //Only one serial port on ProMini.  Skip serial com if Spektrum Sat in use. Note: Spek code will auto-call serialCom if GUI data detected on serial0.
#if defined(GPS_PROMINI)
  if (GPS_Enable == 0) {
    serialCom();
  }
#else
  //serialCom();
#endif
#endif

#if defined(POWERMETER)
  analog.intPowerMeterSum = (pMeter[PMOTOR_SUM] / PLEVELDIV);
  intPowerTrigger1 = conf.powerTrigger1 * PLEVELSCALE;
#endif

#ifdef LCD_TELEMETRY_AUTO
  static char telemetryAutoSequence []  = LCD_TELEMETRY_AUTO;
  static uint8_t telemetryAutoIndex = 0;
  static uint16_t telemetryAutoTimer = 0;
  if ( (telemetry_auto) && (! (++telemetryAutoTimer % LCD_TELEMETRY_AUTO_FREQ) )  ) {
    telemetry = telemetryAutoSequence[++telemetryAutoIndex % strlen(telemetryAutoSequence)];
    LCDclear(); // make sure to clear away remnants
  }
#endif
#ifdef LCD_TELEMETRY
  static uint16_t telemetryTimer = 0;
  if (! (++telemetryTimer % LCD_TELEMETRY_FREQ)) {
#if (LCD_TELEMETRY_DEBUG+0 > 0)
    telemetry = LCD_TELEMETRY_DEBUG;
#endif
    if (telemetry) lcd_telemetry();
  }
#endif

#if GPS & defined(GPS_LED_INDICATOR)       // modified by MIS to use STABLEPIN LED for number of sattelites indication
  static uint32_t GPSLEDTime;              // - No GPS FIX -> LED blink at speed of incoming GPS frames
  static uint8_t blcnt;                    // - Fix and sat no. bellow 5 -> LED off
  if (currentTime > GPSLEDTime) {          // - Fix and sat no. >= 5 -> LED blinks, one blink for 5 sat, two blinks for 6 sat, three for 7 ...
    if (f.GPS_FIX && GPS_numSat >= 5) {
      if (++blcnt > 2 * GPS_numSat) blcnt = 0;
      GPSLEDTime = currentTime + 150000;
      if (blcnt >= 10 && ((blcnt % 2) == 0)) {
        STABLEPIN_ON;
      } else {
        STABLEPIN_OFF;
      }
    } else {
      if ((GPS_update == 1) && !f.GPS_FIX) {
        STABLEPIN_ON;
      } else {
        STABLEPIN_OFF;
      }
      blcnt = 0;
    }
  }
#endif

#if defined(LOG_VALUES) && (LOG_VALUES >= 2)
  if (cycleTime > cycleTimeMax) cycleTimeMax = cycleTime; // remember highscore
  if (cycleTime < cycleTimeMin) cycleTimeMin = cycleTime; // remember lowscore
#endif
  if (f.ARMED)  {
#if defined(LCD_TELEMETRY) || defined(ARMEDTIMEWARNING) || defined(LOG_PERMANENT)
    armedTime += (uint32_t)cycleTime;
#endif
#if defined(VBAT)
    if ( (analog.vbat > NO_VBAT) && (analog.vbat < vbatMin) ) vbatMin = analog.vbat;
#endif
#ifdef LCD_TELEMETRY
#if BARO
    if ( (alt.EstAlt > BAROaltMax) ) BAROaltMax = alt.EstAlt;
#endif
#if GPS
    if ( (GPS_speed > GPS_speedMax) ) GPS_speedMax = GPS_speed;
#endif
#endif
  }
}

static uint32_t RCTime  = 0;
static uint32_t PIDTime = 0;

void setup() {
#if !defined(GPS_PROMINI)
  // Added by Dan, to replace Serial.cpp and Protocol.cpp
  Serial.begin(9600);
  UltrasonicBus.begin(9600);

  // Commented out by Dan
  //    SerialOpen(0,SERIAL0_COM_SPEED);
  //    #if defined(PROMICRO)
  //      SerialOpen(1,SERIAL1_COM_SPEED);
  //    #endif
  //    #if defined(MEGA)
  //      SerialOpen(1,SERIAL1_COM_SPEED);
  //      SerialOpen(2,SERIAL2_COM_SPEED);
  //      SerialOpen(3,SERIAL3_COM_SPEED);
  //    #endif
#endif
  LEDPIN_PINMODE;
  POWERPIN_PINMODE;
  BUZZERPIN_PINMODE;
  STABLEPIN_PINMODE;
  POWERPIN_OFF;
  initOutput();
  readGlobalSet();
#ifndef NO_FLASH_CHECK
#if defined(MEGA)
  uint16_t i = 65000;                             // only first ~64K for mega board due to pgm_read_byte limitation
#else
  uint16_t i = 32000;
#endif
  uint16_t flashsum = 0;
  uint8_t pbyt;
  while (i--) {
    pbyt =  pgm_read_byte(i);        // calculate flash checksum
    flashsum += pbyt;
    flashsum ^= (pbyt << 8);
  }
#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
  global_conf.currentSet = 2;
#else
  global_conf.currentSet = 0;
#endif
  while (1) {                                                   // check settings integrity
#ifndef NO_FLASH_CHECK
    if (readEEPROM()) {                                         // check current setting integrity
      if (flashsum != global_conf.flashsum) update_constants(); // update constants if firmware is changed and integrity is OK
    }
#else
    readEEPROM();                                               // check current setting integrity
#endif
    if (global_conf.currentSet == 0) break;                     // all checks is done
    global_conf.currentSet--;                                   // next setting for check
  }
  readGlobalSet();                              // reload global settings for get last profile number
#ifndef NO_FLASH_CHECK
  if (flashsum != global_conf.flashsum) {
    global_conf.flashsum = flashsum;          // new flash sum
    writeGlobalSet(1);                        // update flash sum in global config
  }
#endif
  readEEPROM();                                 // load setting data from last used profile
  blinkLED(2, 40, global_conf.currentSet + 1);
  configureReceiver();
#if defined (PILOTLAMP)
  PL_INIT;
#endif
  initSensors();
#if defined(I2C_GPS) || defined(GPS_SERIAL) || defined(GPS_FROM_OSD)
  GPS_set_pids();
#endif
  previousTime = micros();
#if defined(GIMBAL)
  calibratingA = 512;
#endif
  calibratingG = 512 + 256;
  calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
#if defined(POWERMETER)
  for (uint8_t j = 0; j <= PMOTOR_SUM; j++) pMeter[j] = 0;
#endif
  /************************************/
#if defined(GPS_SERIAL)
  GPS_SerialInit();
  for (uint8_t j = 0; j <= 5; j++) {
    GPS_NewData();
    LEDPIN_ON
    delay(20);
    LEDPIN_OFF
    delay(80);
  }
  if (!GPS_Present) {
    SerialEnd(GPS_SERIAL);
    SerialOpen(0, SERIAL0_COM_SPEED);
  }
#if !defined(GPS_PROMINI)
  GPS_Present = 1;
#endif
  GPS_Enable = GPS_Present;
#endif
  /************************************/

#if defined(I2C_GPS) || defined(GPS_FROM_OSD)
  GPS_Enable = 1;
#endif

#if defined(LCD_ETPP) || defined(LCD_LCD03) || defined(OLED_I2C_128x64) || defined(OLED_DIGOLE) || defined(LCD_TELEMETRY_STEP)
  initLCD();
#endif
#ifdef LCD_TELEMETRY_DEBUG
  telemetry_auto = 1;
#endif
#ifdef LCD_CONF_DEBUG
  configurationLoop();
#endif
#ifdef LANDING_LIGHTS_DDR
  init_landing_lights();
#endif
#ifdef FASTER_ANALOG_READS
  ADCSRA |= _BV(ADPS2) ; ADCSRA &= ~_BV(ADPS1); ADCSRA &= ~_BV(ADPS0); // this speeds up analogRead without loosing too much resolution: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11
#endif
#if defined(LED_FLASHER)
  init_led_flasher();
  led_flasher_set_sequence(LED_FLASHER_SEQUENCE);
#endif
  f.SMALL_ANGLES_25 = 1; // important for gyro only conf
#ifdef LOG_PERMANENT
  // read last stored set
  readPLog();
  plog.lifetime += plog.armed_time / 1000000;
  plog.start++;         // #powercycle/reset/initialize events
  // dump plog data to terminal
#ifdef LOG_PERMANENT_SHOW_AT_STARTUP
  dumpPLog(0);
#endif
  plog.armed_time = 0;   // lifetime in seconds
  //plog.running = 0;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
#endif

  currentTime = micros();
  RCTime = currentTime;
  PIDTime = currentTime;
  //debugmsg_append_str("initialization completed\n");
}

void go_arm() {
  if (calibratingG == 0
#if defined(ONLYARMWHENFLAT)
      && f.ACC_CALIBRATED
#endif
#if defined(FAILSAFE)
      && failsafeCnt < 2
#endif
     ) {
    if (!f.ARMED && !f.BARO_MODE) { // arm now!
      f.ARMED = 1;
#if defined(HEADFREE) || defined(HEADHOLD)
      headFreeModeHold = att.heading;
      magHold = att.heading;
#else
      headFreeModeHold = att.heading;
      magHold = att.heading;
#endif
#if defined(VBAT)
      if (analog.vbat > NO_VBAT) vbatMin = analog.vbat;
#endif
#ifdef LCD_TELEMETRY // reset some values when arming
#if BARO
      BAROaltMax = alt.EstAlt;
#endif
#if GPS
      GPS_speedMax = 0;
#endif
#ifdef POWERMETER_HARD
      powerValueMaxMAH = 0;
#endif
#endif
#ifdef LOG_PERMANENT
      plog.arm++;           // #arm events
      plog.running = 1;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
      // write now.
      writePLog();
#endif
    }
  } else if (!f.ARMED) {
    blinkLED(2, 255, 1);
    alarmArray[8] = 1;
  }
}
void go_disarm() {
  if (f.ARMED) {
    f.ARMED = 0;
#ifdef LOG_PERMANENT
    plog.disarm++;        // #disarm events
    plog.armed_time = armedTime ;   // lifetime in seconds
    if (failsafeEvents) plog.failsafe++;      // #acitve failsafe @ disarm
    if (i2c_errors_count > 10) plog.i2c++;           // #i2c errs @ disarm
    plog.running = 0;       // toggle @ arm & disarm to monitor for clean shutdown vs. powercut
    // write now.
    writePLog();
#endif
  }
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

int doRange(byte Address) {
  SRF01_Cmd(Address, getRange);                              // Calls a function to get range from SRF01
  while (UltrasonicBus.available() < 2);                     // Waits to get good data
  byte highByte = UltrasonicBus.read();                      // Get high byte
  byte lowByte = UltrasonicBus.read();                       // Get low byte
  int dist = ((highByte << 8) + lowByte);                    // Put them together
  return dist;
}


// ******** Main Loop *********
void loop () {
  static uint8_t rcDelayCommand; // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
  static uint8_t rcSticks;       // this hold sticks position for command combos
  uint8_t axis, i;
  int16_t error, errorAngle;
  int16_t delta;
  int16_t PTerm = 0, ITerm = 0, DTerm, PTermACC, ITermACC;
  static int16_t lastGyro[2] = {0, 0};
  static int16_t errorAngleI[2] = {0, 0};
  static int16_t delta1[3], delta2[3];
  static int32_t errorGyroI[3] = {0, 0, 0};
  static int16_t lastError[3] = {0, 0, 0};
  int16_t deltaSum;
  int16_t AngleRateTmp, RateError;
  static int16_t initialThrottleHold;
  static uint32_t timestamp_fixated = 0;
  int16_t rc;
  int32_t prop = 0;
  unsigned long currentMillis = millis();

  // This should be Wire stuff, you fucks. - Dan
  while (Serial.available()) {
    //int inChar = Serial.read();
    for (int i = 0; i < 4; i++) {
      //      reader[i] = Serial.read(); //added by Dan, in place of all the casting
      tmpint = Serial.read(); //use this with Kaas' computer, if we have to
      tmpstr = (char)tmpint;
      reader[i] = tmpstr.toInt();
      //      inString += (char)inChar;
      //      readIn = inString.toInt();
      delay(2);
    }
  }


  //Only listen to the Ultrasonic on every 150th iteration
  UltrasonicBus.listen();
  
  if (ultraCount > 50) {
    if (UltrasonicBus.isListening()) {
      int dist = doRange(srfAddress1);
      distAVG += dist;
    }
//    thrErr += 3*(setpoint - distAVG/100);
    thrErr += 25 * (setpoint - distAVG);
    if ( thrErr > 1999 ) thrErr = 1999;
    if ( thrErr < 1501 ) thrErr = 1501;

    //        delay(20);
    conf.throttleIn = thrErr;

    ultraCount = 0;
    distAVG = 0;
  }

  ultraCount++;

  //    else if (dist > setpoint && c > 1000) {
  //      c -= 1.5*abs(setpoint - dist);
  ////      Serial.print(c);m
  //      delay(20);
  //    }



  //    Serial.print(c);
  //    Serial.println();



  if (reader[0] == 9) {
    if (!f.ARMED) f.ARMED = 1;
    else if (f.ARMED) f.ARMED = 0;
    conf.throttleIn = 1000;
    calibratingA=512;
  }


  /***************************************CONTROL SEQUENCE******************************************/

  //roll 0, pitch 1, yaw 2

  /*
    Serial.print("roll =   ");
    Serial.print(imu.gyroADC[0]);
    Serial.print("   ");
    Serial.print("pitch =   ");
    Serial.print(imu.gyroADC[1]);
    Serial.print("   ");
    Serial.print("yaw =   ");
    Serial.print(imu.gyroADC[2]);
    Serial.println("   ");


    if(counter == 10){
      rollZero = imu.gyroADC[0];
      pitchZero = imu.gyroADC[1];
      yawZero = imu.gyroADC[2];
    } counter++;

    rollErr = 0.01*(rollZero - imu.gyroADC[0]);
    pitchErr = (pitchZero - imu.gyroADC[1]);

    if(rollErr > 1999) rollErr = 1999;
    if(rollErr < 1101) rollErr = 1101;

    if(pitchErr > 1999) pitchErr = 1999;
    if(pitchErr < 1101) pitchErr = 1101;

  */
  //Serial.print(rollZero); Serial.print("    "); Serial.print(imu.gyroADC[0]); Serial.print("    "); Serial.print(rollErr);  Serial.println();

  //  conf.throttleIn = 1200;
  //  conf.rollIn = 1200;   //1500+ right; 1500- left
  //  conf.pitchIn  = 1200;   //1500+ forward; 1500- backward


  //  if(reader[0]==1){
  //if(readIn > 1000 && readIn < 2001) {
  //    conf.throttleIn = reader[0]*1000+reader[1]*100+reader[2]*10+reader[3];

  //conf.throttleIn = readIn;
  //inString = "";
  //  }

  //  if(reader[0]==2){
  //    reader[0]=reader[0]-1;
  //    conf.rollIn = reader[0]*1000+reader[1]*100+reader[2]*10+reader[3];
  //  }
  //
  //  if(reader[0]==3){
  //    reader[0]=reader[0]-2;
  //    conf.pitchIn = reader[0]*1000+reader[1]*100+reader[2]*10+reader[3];
  //  }

  for (int i = 0; i < 4; i++) reader[i] = 0;

#if defined(SPEKTRUM)
  if (spekFrameFlags == 0x01) readSpektrum();
#endif

  currentTime = micros();

  if (currentTime > RCTime ) { // 50Hz
    RCTime += CHECK_RCTIME;
    computeRC();
    // Commented out by Dan, to allow arming by software
    // Failsafe routine - added by MIS
#if defined(FAILSAFE)
    if ( failsafeCnt > (5 * FAILSAFE_DELAY) && f.ARMED) {                // Stabilize, and set Throttle to specified level
      for (i = 0; i < 3; i++) rcData[i] = MIDRC;                          // after specified guard time after RC signal is lost (in 0.1sec)
      rcData[THROTTLE] = conf.failsafe_throttle;
      if (failsafeCnt > 5 * (FAILSAFE_DELAY + FAILSAFE_OFF_DELAY)) {      // Turn OFF motors after specified Time (in 0.1sec)
        go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents - Commented out by Dan
        f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
      }
      failsafeEvents++;
    }
    if ( failsafeCnt > (5 * FAILSAFE_DELAY) && !f.ARMED) { //Turn of "Ok To arm to prevent the motors from spinning after repowering the RX with low throttle and aux to arm
      go_disarm();     // This will prevent the copter to automatically rearm if failsafe shuts it down and prevents
      f.OK_TO_ARM = 0; // to restart accidentely by just reconnect to the tx - you will have to switch off first to rearm
    }
    failsafeCnt++;
#endif
    // end of failsafe routine - next change is made with RcOptions setting

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    uint8_t stTmp = 0;
    for (i = 0; i < 4; i++) {
      stTmp >>= 2;
      if (rcData[i] > MINCHECK) stTmp |= 0x80;     // check for MIN
      if (rcData[i] < MAXCHECK) stTmp |= 0x40;     // check for MAX
    }
    if (stTmp == rcSticks) {
      if (rcDelayCommand < 250) rcDelayCommand++;
    } else rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (rcData[THROTTLE] <= MINCHECK) {            // THROTTLE at minimum
#if !defined(FIXEDWING)
      errorGyroI[ROLL] = 0;
      errorGyroI[PITCH] = 0;
      errorGyroI[YAW] = 0;
      errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
#endif
      if (conf.activate[BOXARM] > 0) {             // Arming/Disarming via ARM BOX
        if ( rcOptions[BOXARM] && f.OK_TO_ARM ) go_arm(); else if (f.ARMED) go_disarm();
      }
    }
    if (rcDelayCommand == 20) {
      if (f.ARMED) {                  // actions during armed
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        //if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) go_disarm();    // Disarm via YAW
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        //if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO) go_disarm();    // Disarm via ROLL
#endif
      } else {                        // actions during not armed
        i = 0;
        if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {    // GYRO calibration
          calibratingG = 512;
#if GPS
          GPS_reset_home_position();
#endif
#if BARO
          calibratingB = 10; // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
#endif
        }
#if defined(INFLIGHT_ACC_CALIBRATION)
        else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI) {    // Inflight ACC calibration START/STOP
          if (AccInflightCalibrationMeasurementDone) {               // trigger saving into eeprom after landing
            AccInflightCalibrationMeasurementDone = 0;
            AccInflightCalibrationSavetoEEProm = 1;
          } else {
            AccInflightCalibrationArmed = !AccInflightCalibrationArmed;
          }
        }
#endif
#ifdef MULTIPLE_CONFIGURATION_PROFILES
        if      (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO) i = 1;  // ROLL left  -> Profile 1
        else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE) i = 2;  // PITCH up   -> Profile 2
        else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI) i = 3;  // ROLL right -> Profile 3
        if (i) {
          global_conf.currentSet = i - 1;
          writeGlobalSet(0);
          readEEPROM();
          blinkLED(2, 40, i);
          alarmArray[0] = i;
        }
#endif
        if (rcSticks == THR_LO + YAW_HI + PIT_HI + ROL_CE) {            // Enter LCD config
#if defined(LCD_CONF)
          configurationLoop(); // beginning LCD configuration
#endif
          previousTime = micros();
        }
#ifdef ALLOW_ARM_DISARM_VIA_TX_YAW
        else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) go_arm();      // Arm via YAW
#endif
#ifdef ALLOW_ARM_DISARM_VIA_TX_ROLL
        else if (conf.activate[BOXARM] == 0 && rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI) go_arm();      // Arm via ROLL
#endif
#ifdef LCD_TELEMETRY_AUTO
        else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {              // Auto telemetry ON/OFF
          if (telemetry_auto) {
            telemetry_auto = 0;
            telemetry = 0;
          } else
            telemetry_auto = 1;
        }
#endif
#ifdef LCD_TELEMETRY_STEP
        else if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {              // Telemetry next step
          telemetry = telemetryStepSequence[++telemetryStepIndex % strlen(telemetryStepSequence)];
#if defined( OLED_I2C_128x64)
          if (telemetry != 0) i2c_OLED_init();
#elif defined(OLED_DIGOLE)
          if (telemetry != 0) i2c_OLED_DIGOLE_init();
#endif
          LCDclear();
        }
#endif
#if ACC
        else if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) calibratingA = 512;   // throttle=max, yaw=left, pitch=min
#endif
#if MAG
        else if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) f.CALIBRATE_MAG = 1;  // throttle=max, yaw=right, pitch=min
#endif
        i = 0;
        if      (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
          conf.angleTrim[PITCH] += 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
          conf.angleTrim[PITCH] -= 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
          conf.angleTrim[ROLL] += 2;
          i = 1;
        }
        else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
          conf.angleTrim[ROLL] -= 2;
          i = 1;
        }
        if (i) {
          writeParams(1);
          rcDelayCommand = 0;    // allow autorepetition
#if defined(LED_RING)
          blinkLedRing();
#endif
        }
      }
    }
#if defined(LED_FLASHER)
    led_flasher_autoselect_sequence();
#endif

#if defined(INFLIGHT_ACC_CALIBRATION)
    if (AccInflightCalibrationArmed && f.ARMED && rcData[THROTTLE] > MINCHECK && !rcOptions[BOXARM] ) { // Copter is airborne and you are turning it off via boxarm : start measurement
      InflightcalibratingA = 50;
      AccInflightCalibrationArmed = 0;
    }
    if (rcOptions[BOXCALIB]) {      // Use the Calib Option to activate : Calib = TRUE Meausrement started, Land and Calib = 0 measurement stored
      if (!AccInflightCalibrationActive && !AccInflightCalibrationMeasurementDone) {
        InflightcalibratingA = 50;
      }
    } else if (AccInflightCalibrationMeasurementDone && !f.ARMED) {
      AccInflightCalibrationMeasurementDone = 0;
      AccInflightCalibrationSavetoEEProm = 1;
    }
#endif

    uint16_t auxState = 0;
    for (i = 0; i < 4; i++)
      auxState |= (rcData[AUX1 + i] < 1300) << (3 * i) | (1300 < rcData[AUX1 + i] && rcData[AUX1 + i] < 1700) << (3 * i + 1) | (rcData[AUX1 + i] > 1700) << (3 * i + 2);
    for (i = 0; i < CHECKBOXITEMS; i++)
      rcOptions[i] = (auxState & conf.activate[i]) > 0;

    // note: if FAILSAFE is disable, failsafeCnt > 5*FAILSAFE_DELAY is always false
#if ACC
    if ( rcOptions[BOXANGLE] || (failsafeCnt > 5 * FAILSAFE_DELAY) ) {
      // bumpless transfer to Level mode
      if (!f.ANGLE_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.ANGLE_MODE = 1;
      }
    } else {
      // failsafe support
      f.ANGLE_MODE = 0;
    }
    if ( rcOptions[BOXHORIZON] ) {
      f.ANGLE_MODE = 0;
      if (!f.HORIZON_MODE) {
        errorAngleI[ROLL] = 0; errorAngleI[PITCH] = 0;
        f.HORIZON_MODE = 1;
      }
    } else {
      f.HORIZON_MODE = 0;
    }
#endif

#ifdef HEADFREE
    if (rcOptions[BOXHEADFREE] == 1) f.HEADFREE_MODE = 1; else f.HEADFREE_MODE = 0;
#endif

#ifdef HEADHOLD
    if (rcOptions[BOXHEADHOLD] == 1) f.HEADHOLD_MODE = 1; else f.HEADHOLD_MODE = 0;
#endif

    if (rcOptions[BOXARM] == 0) f.OK_TO_ARM = 1;
#if !defined(GPS_LED_INDICATOR)
    if (f.ANGLE_MODE || f.HORIZON_MODE) {
      STABLEPIN_ON;
    } else {
      STABLEPIN_OFF;
    }
#endif

#if defined(PID_SWITCH)
    {
      boolean changed = false;
      if (rcOptions[BOXPID]) {
        if (!f.PID_MODE) {
          f.PID_MODE = 1;
          changed = true;
        }
      } else {
        if ( f.PID_MODE) {
          f.PID_MODE = 0;
          changed = true;
        }
      }
      if (changed) {
        conf.pid[PIDROLL].P8  = conf.pidset[f.PID_MODE][PID_ROLL].P8;
        conf.pid[PIDROLL].I8  = conf.pidset[f.PID_MODE][PID_ROLL].I8;
        conf.pid[PIDROLL].D8  = conf.pidset[f.PID_MODE][PID_ROLL].D8;
        conf.pid[PIDPITCH].P8 = conf.pidset[f.PID_MODE][PID_PITCH].P8;
        conf.pid[PIDPITCH].I8 = conf.pidset[f.PID_MODE][PID_PITCH].I8;
        conf.pid[PIDPITCH].D8 = conf.pidset[f.PID_MODE][PID_PITCH].D8;
        conf.pid[PIDLEVEL].P8 = conf.pidset[f.PID_MODE][PID_LEVEL].P8;
        // I term in oldskool, Horizon P Term in alexk
        conf.pid[PIDLEVEL].I8 = conf.pidset[f.PID_MODE][PID_LEVEL].I8;
        // not D term, but windup limit in oldskool, nothing in alexk
        conf.pid[PIDLEVEL].D8 = conf.pidset[f.PID_MODE][PID_LEVEL].D8;
      }
    }
#else
    if (PID_CONTROLLER == 1) f.PID_MODE = 0;
    else f.PID_MODE = 1;
#endif

    //debug[0] = analog.vbat;
    //debug[1] = analogRead(A2);
    //debug[2] = analogRead(A4);

    /*
         Ziegler–Nichols method Control Type
              Kp      Ki         Kd
          P   0.50Ku    -        -
          PI  0.45Ku  1.2 Kp/Pu  -
          PID 0.60Ku  2 Kp/Pu    KpPu/8

          Ku = input waarde 0...250
          Kp = berekende P waarde (0,6 Ku)
          Pu = de frequentie van oscillatie
    */

#if defined(DIAL_TUNING)
    int ku;
    int kp;
#if defined(POT_PR)
    ku = ((constrain(rcData[POT_PR], 1000, 2000) - 1000) << 1) >> 6 ; // * 20/640;
    kp = ku * 3 ;
    conf.pid[PIDROLL].P8 = kp;
    conf.pid[PIDROLL].I8 = (kp << 1) / FREQ_TU_PR;
    conf.pid[PIDROLL].D8 = (kp * FREQ_TU_PR) >> 3;
    conf.pid[PIDPITCH].P8 = kp;
    conf.pid[PIDPITCH].I8 = (kp << 1) / FREQ_TU_PR;
    conf.pid[PIDPITCH].D8 = (kp * FREQ_TU_PR) >> 3;
#endif
#if defined(POT_L)
    ku = ((constrain(rcData[POT_L], 1000, 2000) - 1000) << 1) >> 6 ; // * 20/640;
    kp = ku * 4 ;
    conf.pid[PIDLEVEL].P8 = kp;
    if (f.PID_MODE == 0)
      conf.pid[PIDLEVEL].I8 = (kp << 1) / FREQ_TU_PR;
    else
      conf.pid[PIDLEVEL].I8 = kp;
#endif
#endif

#if BARO
#if (!defined(SUPPRESS_BARO_ALTHOLD))
    if (rcOptions[BOXBARO]) {
      if (!f.BARO_MODE) {
        f.BARO_MODE = 1;
        AltHold = alt.EstAlt;
#if defined(ALT_HOLD_THROTTLE_MIDPOINT)
        initialThrottleHold = ALT_HOLD_THROTTLE_MIDPOINT;
#else
        initialThrottleHold = rcCommand[THROTTLE];
#endif
        errorAltitudeI = 0;
        BaroPID = 0;
      }
    } else {
      f.BARO_MODE = 0;
    }
#endif
#ifdef VARIOMETER
    if (rcOptions[BOXVARIO]) {
      if (!f.VARIO_MODE) {
        f.VARIO_MODE = 1;
      }
    } else {
      f.VARIO_MODE = 0;
    }
#endif
#endif
#if MAG
    if (rcOptions[BOXMAG]) {
      if (!f.MAG_MODE) {
        f.MAG_MODE = 1;
        magHold = att.heading;
      }
    } else {
      f.MAG_MODE = 0;
    }
    if (rcOptions[BOXHEADFREE]) {
      if (!f.HEADFREE_MODE) {
        f.HEADFREE_MODE = 1;
      }
#if defined(ADVANCED_HEADFREE)
      if ((f.GPS_FIX && GPS_numSat >= 5) && (GPS_distanceToHome > ADV_HEADFREE_RANGE) ) {
        if (GPS_directionToHome < 180)  {
          headFreeModeHold = GPS_directionToHome + 180;
        } else {
          headFreeModeHold = GPS_directionToHome - 180;
        }
      }
#endif
    } else {
      f.HEADFREE_MODE = 0;
    }
    if (rcOptions[BOXHEADADJ]) {
      headFreeModeHold = att.heading; // acquire new heading
    }
#endif

#if GPS
    static uint8_t GPSNavReset = 1;
    if (f.GPS_FIX && GPS_numSat >= 5 ) {
      if (rcOptions[BOXGPSHOME]) {  // if both GPS_HOME & GPS_HOLD are checked => GPS_HOME is the priority
        if (!f.GPS_HOME_MODE)  {
          f.GPS_HOME_MODE = 1;
          f.GPS_HOLD_MODE = 0;
          GPSNavReset = 0;
#if defined(I2C_GPS)
          GPS_I2C_command(I2C_GPS_COMMAND_START_NAV, 0);       //waypoint zero
#else // SERIAL
          GPS_set_next_wp(&GPS_home[LAT], &GPS_home[LON]);
          nav_mode    = NAV_MODE_WP;
#endif
        }
      } else {
        f.GPS_HOME_MODE = 0;
        if (rcOptions[BOXGPSHOLD] && abs(rcCommand[ROLL]) < AP_MODE && abs(rcCommand[PITCH]) < AP_MODE) {
          if (!f.GPS_HOLD_MODE) {
            f.GPS_HOLD_MODE = 1;
            GPSNavReset = 0;
#if defined(I2C_GPS)
            GPS_I2C_command(I2C_GPS_COMMAND_POSHOLD, 0);
#else
            GPS_hold[LAT] = GPS_coord[LAT];
            GPS_hold[LON] = GPS_coord[LON];
            GPS_set_next_wp(&GPS_hold[LAT], &GPS_hold[LON]);
            nav_mode = NAV_MODE_POSHOLD;
#endif
          }
        } else {
          f.GPS_HOLD_MODE = 0;
          // both boxes are unselected here, nav is reset if not already done
          if (GPSNavReset == 0 ) {
            GPSNavReset = 1;
            GPS_reset_nav();
          }
        }
      }
    } else {
      f.GPS_HOME_MODE = 0;
      f.GPS_HOLD_MODE = 0;
#if !defined(I2C_GPS)
      nav_mode = NAV_MODE_NONE;
#endif
    }
#endif

#if defined(FIXEDWING) || defined(HELICOPTER)
    if (rcOptions[BOXPASSTHRU]) {
      f.PASSTHRU_MODE = 1;
    }
    else {
      f.PASSTHRU_MODE = 0;
    }
#endif

  } else { // not in rc loop
    static uint8_t taskOrder = 0; // never call all functions in the same loop, to avoid high delay spikes
    if (taskOrder > 4) taskOrder -= 5;
    switch (taskOrder) {
      case 0:
        taskOrder++;
#if MAG
        if (Mag_getADC()) break; // max 350 µs (HMC5883) // only break when we actually did something
#endif
      case 1:
        taskOrder++;
#if BARO
        if (Baro_update() != 0 ) break;
#endif
      case 2:
        taskOrder++;
#if BARO
        if (getEstimatedAltitude() != 0) break;
#endif
      case 3:
        taskOrder++;
#if GPS
        if (GPS_Enable) GPS_NewData();
        break;
#endif
      case 4:
        taskOrder++;
#if SONAR
        Sonar_update(); //debug[2] = sonarAlt;
#endif
#ifdef LANDING_LIGHTS_DDR
        auto_switch_landing_lights();
#endif
#ifdef VARIOMETER
        if (f.VARIO_MODE) vario_signaling();
#endif
        break;
    }
  }

  if (currentTime >= PIDTime)
  {
    PIDTime += PIDCALCTIME;

    computeIMU();

    // Measure loop rate just afer reading the sensors
    cycleTime = currentTime - previousTime;
    previousTime = currentTime;

    //***********************************
    //**** Experimental FlightModes *****
    //***********************************
#if defined(ACROTRAINER_MODE)
    if (f.ANGLE_MODE) {
      if (abs(rcCommand[ROLL]) + abs(rcCommand[PITCH]) >= ACROTRAINER_MODE ) {
        f.ANGLE_MODE = 1; //changed to 1 by Alex
        f.HORIZON_MODE = 0; //changed to 1 by Alex
        f.MAG_MODE = 0;
        f.BARO_MODE = 0;
        f.GPS_HOME_MODE = 0;
        f.GPS_HOLD_MODE = 0;
      }
    }
#endif

#ifdef HEADHOLD
    if (f.HEADHOLD_MODE)
    {
      errorAngle = headFreeModeHold - att.heading;
      if (errorAngle >  180) errorAngle -= 360;
      if (errorAngle < -180) errorAngle += 360;
      errorAngle = constrain(errorAngle, -100, 100);
      rcCommand[YAW] += (errorAngle * conf.pid[PIDMAG].P8) >> 4;
    }
#endif

    //***********************************

#if MAG
    if (abs(rcCommand[YAW]) < 70 && f.MAG_MODE) {
      int16_t dif = att.heading - magHold;
      if (dif <= - 180) dif += 360;
      if (dif >= + 180) dif -= 360;
      if ( f.SMALL_ANGLES_25 ) rcCommand[YAW] -= dif * conf.pid[PIDMAG].P8 >> 5;
    } else magHold = att.heading;
#endif

#if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    /* Smooth alt change routine , for slow auto and aerophoto modes (in general solution from alexmos). It's slowly increase/decrease
       altitude proportional to stick movement (+/-100 throttle gives about +/-50 cm in 1 second with cycle time about 3-4ms)
    */
    if (f.BARO_MODE) {
      static uint8_t isAltHoldChanged = 0;
      static int16_t AltHoldCorr = 0;
      if (abs(rcCommand[THROTTLE] - initialThrottleHold) > ALT_HOLD_THROTTLE_NEUTRAL_ZONE) {
        // Slowly increase/decrease AltHold proportional to stick movement ( +100 throttle gives ~ +50 cm in 1 second with cycle time about 3-4ms)
        AltHoldCorr += rcCommand[THROTTLE] - initialThrottleHold;
        if (abs(AltHoldCorr) > 512) {
          AltHold += AltHoldCorr / 512;
          AltHoldCorr %= 512;
        }
        isAltHoldChanged = 1;
      } else if (isAltHoldChanged) {
        AltHold = alt.EstAlt;
        isAltHoldChanged = 0;
      }
      rcCommand[THROTTLE] = initialThrottleHold + BaroPID;
    }
#endif

#if defined(THROTTLE_ANGLE_CORRECTION)
    if (f.ANGLE_MODE || f.HORIZON_MODE) {
      rcCommand[THROTTLE] += throttleAngleCorrection;
    }
#endif

#if GPS
    if ( (f.GPS_HOME_MODE || f.GPS_HOLD_MODE) && f.GPS_FIX_HOME ) {
      float sin_yaw_y = sin(att.heading * 0.0174532925f);
      float cos_yaw_x = cos(att.heading * 0.0174532925f);
#if defined(NAV_SLEW_RATE)
      nav_rated[LON]   += constrain(wrap_18000(nav[LON] - nav_rated[LON]), -NAV_SLEW_RATE, NAV_SLEW_RATE);
      nav_rated[LAT]   += constrain(wrap_18000(nav[LAT] - nav_rated[LAT]), -NAV_SLEW_RATE, NAV_SLEW_RATE);
      GPS_angle[ROLL]   = (nav_rated[LON] * cos_yaw_x - nav_rated[LAT] * sin_yaw_y) / 10;
      GPS_angle[PITCH]  = (nav_rated[LON] * sin_yaw_y + nav_rated[LAT] * cos_yaw_x) / 10;
#else
      GPS_angle[ROLL]   = (nav[LON] * cos_yaw_x - nav[LAT] * sin_yaw_y) / 10;
      GPS_angle[PITCH]  = (nav[LON] * sin_yaw_y + nav[LAT] * cos_yaw_x) / 10;
#endif
    } else {
      GPS_angle[ROLL]  = 0;
      GPS_angle[PITCH] = 0;
    }
#endif

    //**** PITCH & ROLL & YAW PID ****
    if (f.PID_MODE == 0) { // evolved oldschool  **********************

      if ( f.HORIZON_MODE ) prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 512);

      // PITCH & ROLL
      for (axis = 0; axis < 2; axis++) {
        rc = rcCommand[axis] << 1;
        error = rc - imu.gyroData[axis];
        errorGyroI[axis]  = constrain(errorGyroI[axis] + error, -16000, +16000);   // WindUp   16 bits is ok here
        if (abs(imu.gyroData[axis]) > 640) errorGyroI[axis] = 0;

        ITerm = (errorGyroI[axis] >> 7) * conf.pid[axis].I8 >> 6;                  // 16 bits is ok here 16000/125 = 128 ; 128*250 = 32000

        PTerm = (int32_t)rc * conf.pid[axis].P8 >> 6;

        if (f.ANGLE_MODE || f.HORIZON_MODE) { // axis relying on ACC
          // 50 degrees max inclination
          errorAngle         = constrain(rc + GPS_angle[axis], -500, +500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
          errorAngleI[axis]  = constrain(errorAngleI[axis] + errorAngle, -10000, +10000);                                            // WindUp     //16 bits is ok here

          PTermACC           = ((int32_t)errorAngle * conf.pid[PIDLEVEL].P8) >> 7; // 32 bits is needed for calculation: errorAngle*P8 could exceed 32768   16 bits is ok for result

          int16_t limit      = conf.pid[PIDLEVEL].D8 * 5;
          PTermACC           = constrain(PTermACC, -limit, +limit);

          ITermACC           = ((int32_t)errorAngleI[axis] * conf.pid[PIDLEVEL].I8) >> 12; // 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

          ITerm              = ITermACC + ((ITerm - ITermACC) * prop >> 9);
          PTerm              = PTermACC + ((PTerm - PTermACC) * prop >> 9);
        }

        PTerm -= ((int32_t)imu.gyroData[axis] * dynP8[axis]) >> 6; // 32 bits is needed for calculation

        delta          = imu.gyroData[axis] - lastGyro[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastGyro[axis] = imu.gyroData[axis];
        DTerm          = delta1[axis] + delta2[axis] + delta;
        delta2[axis]   = delta1[axis];
        delta1[axis]   = delta;

        DTerm = ((int32_t)DTerm * dynD8[axis]) >> 5;    // 32 bits is needed for calculation

        axisPID[axis] =  PTerm + ITerm - DTerm;
      }

      //YAW
#define GYRO_P_MAX 300
#define GYRO_I_MAX 250

      rc = (int32_t)rcCommand[YAW] * (2 * conf.yawRate + 30)  >> 5;

      error = rc - imu.gyroData[YAW];
      errorGyroI[YAW]  += (int32_t)error * conf.pid[PIDYAW].I8;
      errorGyroI[YAW]  = constrain(errorGyroI[YAW], 2 - ((int32_t)1 << 28), -2 + ((int32_t)1 << 28));
      if (abs(rc) > 50) errorGyroI[YAW] = 0;

      PTerm = (int32_t)error * conf.pid[PIDYAW].P8 >> 6;
      int16_t limit = GYRO_P_MAX - conf.pid[PIDYAW].D8;
      PTerm = constrain(PTerm, -limit, +limit);

      ITerm = constrain((int16_t)(errorGyroI[YAW] >> 13), -GYRO_I_MAX, +GYRO_I_MAX);

      axisPID[YAW] =  PTerm + ITerm;

    } else { // AlexK *******************

      // Angle is Level_P
      // Horizon is Level_I
      // Level_D is unused

#define GYRO_I_MAX 256
#define ACC_I_MAX 256
      prop = min(max(abs(rcCommand[PITCH]), abs(rcCommand[ROLL])), 500); // range [0;500]

      //----------PID controller----------
      for (axis = 0; axis < 3; axis++) {
        //-----Get the desired angle rate depending on flight mode
        if ((f.ANGLE_MODE || f.HORIZON_MODE) && axis < 2 ) { // MODE relying on ACC
          // calculate error and limit the angle to 50 degrees max inclination
          errorAngle = constrain((rcCommand[axis] << 1) + GPS_angle[axis], -500, +500) - att.angle[axis] + conf.angleTrim[axis]; //16 bits is ok here
        }
        if (axis == 2) {//YAW is always gyro-controlled (MAG correction is applied to rcCommand)
          AngleRateTmp = (((int32_t) (conf.yawRate + 27) * rcCommand[2]) >> 5);
        } else {
          if (!f.ANGLE_MODE) {//control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
            AngleRateTmp = ((int32_t) (conf.rollPitchRate + 27) * rcCommand[axis]) >> 4;
            if (f.HORIZON_MODE) {
              //mix up angle error to desired AngleRateTmp to add a little auto-level feel
              AngleRateTmp += ((int32_t) errorAngle * conf.pid[PIDLEVEL].I8) >> 8;
            }
          } else {//it's the ANGLE mode - control is angle based, so control loop is needed
            AngleRateTmp = ((int32_t) errorAngle * conf.pid[PIDLEVEL].P8) >> 4;
          }
        }

        //--------low-level gyro-based PID. ----------
        //Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        //-----calculate scaled error.AngleRates
        //multiplication of rcCommand corresponds to changing the sticks scaling here
        RateError = AngleRateTmp  - imu.gyroData[axis];

        //-----calculate P component
        PTerm = ((int32_t) RateError * conf.pid[axis].P8) >> 7;

        //-----calculate I component
        //there should be no division before accumulating the error to integrator, because the precision would be reduced.
        //Precision is critical, as I prevents from long-time drift. Thus, 32 bits integrator is used.
        //Time correction (to avoid different I scaling for different builds based on average cycle time)
        //is normalized to cycle time = 2048.
        errorGyroI[axis]  += (((int32_t) RateError * cycleTime) >> 11) * conf.pid[axis].I8;
        //limit maximum integrator value to prevent WindUp - accumulating extreme values when system is saturated.
        //I coefficient (I8) moved before integration to make limiting independent from PID settings
        errorGyroI[axis]  = constrain(errorGyroI[axis], (int32_t) - GYRO_I_MAX << 13, (int32_t) + GYRO_I_MAX << 13);
        ITerm = errorGyroI[axis] >> 13;

        //-----calculate D-term
        delta          = RateError - lastError[axis];  // 16 bits is ok here, the dif between 2 consecutive gyro reads is limited to 800
        lastError[axis] = RateError;

        //Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta = ((int32_t) delta * ((uint16_t)0xFFFF / (cycleTime >> 4))) >> 6;
        //add moving average here to reduce noise
        deltaSum       = delta1[axis] + delta2[axis] + delta;
        delta2[axis]   = delta1[axis];
        delta1[axis]   = delta;

        DTerm = (deltaSum * conf.pid[axis].D8) >> 8;

        //-----calculate total PID output
        axisPID[axis] =  PTerm + ITerm + DTerm;
      }

    } // f.PID_MODE **********************

    mixTable();
    // do not update servos during unarmed calibration of sensors which are sensitive to vibration
    if ( (f.ARMED) || ((!calibratingG) && (!calibratingA)) ) writeServos();
    writeMotors();
    /*if(!f.ARMED){
      f.ARMED = 1; //Alex (ok... Daniel)
      }*/
  }
}
