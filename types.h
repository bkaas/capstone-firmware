#ifndef TYPES_H_
#define TYPES_H_

enum rc {
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX1,
  AUX2,
  AUX3,
  AUX4,
  AUX5,
  AUX6,
  AUX7,
  AUX8
};

enum pid {
  PIDROLL,
  PIDPITCH,
  PIDYAW,
  PIDALT,
  PIDPOS,
  PIDPOSR,
  PIDNAVR,
  PIDLEVEL,
  PIDMAG,
  PIDVEL,     // not used currently
  PIDITEMS
};

#define NUMPIDSETS 2
enum pid_set {
  PID_ROLL,
  PID_PITCH,
  PID_LEVEL,
  PIDSETITEMS
};

enum box {
  BOXARM,
  #if ACC
    BOXANGLE,
    BOXHORIZON,
  #endif
  #if BARO && (!defined(SUPPRESS_BARO_ALTHOLD))
    BOXBARO,
  #endif
  #ifdef VARIOMETER
    BOXVARIO,
  #endif
  
  #ifdef HEADFREE
	BOXHEADFREE,
  #endif
  #ifdef HEADHOLD
	BOXHEADHOLD,
  #endif
  
  #if MAG
    BOXMAG,
    BOXHEADFREE,
    BOXHEADADJ, // acquire heading for HEADFREE mode
  #endif
  #if defined(SERVO_TILT) || defined(GIMBAL)  || defined(SERVO_MIX_TILT)
    BOXCAMSTAB,
  #endif
  #if defined(CAMTRIG)
    BOXCAMTRIG,
  #endif
  #if GPS
    BOXGPSHOME,
    BOXGPSHOLD,
  #endif
  #if defined(FIXEDWING) || defined(HELICOPTER)
    BOXPASSTHRU,
  #endif
  #if defined(BUZZER)
    BOXBEEPERON,
  #endif
  #if defined(LED_FLASHER)
    BOXLEDMAX, // we want maximum illumination
    BOXLEDLOW, // low/no lights
  #endif
  #if defined(LANDING_LIGHTS_DDR)
    BOXLLIGHTS, // enable landing lights at any altitude
  #endif
  #ifdef INFLIGHT_ACC_CALIBRATION
    BOXCALIB,
  #endif
  #ifdef PID_SWITCH
    BOXPID,
  #endif
  #ifdef OSD_SWITCH
    BOXOSD,
  #endif
  CHECKBOXITEMS
};

typedef struct {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
} imu_t;

typedef struct {
  uint8_t  vbat;               // battery voltage in 0.1V steps
  uint16_t intPowerMeterSum;
  uint16_t rssi;              // range: [0;1023]
  uint16_t amperage;
} analog_t;

typedef struct {
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
} alt_t;

typedef struct {
  int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
  int16_t heading;             // variometer in cm/s
} att_t;

typedef struct {
  uint8_t OK_TO_ARM :1 ;
  uint8_t ARMED :1 ;
  uint8_t I2C_INIT_DONE :1 ; // For i2c gps we have to now when i2c init is done, so we can update parameters to the i2cgps from eeprom (at startup it is done in setup())
  uint8_t ACC_CALIBRATED :1 ;
  uint8_t NUNCHUKDATA :1 ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t MAG_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t GPS_HOME_MODE :1 ;
  uint8_t GPS_HOLD_MODE :1 ;
  uint8_t HEADFREE_MODE :1 ;
  uint8_t HEADHOLD_MODE :1 ;
  uint8_t PASSTHRU_MODE :1 ;
  uint8_t GPS_FIX :1 ;
  uint8_t GPS_FIX_HOME :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t CALIBRATE_MAG :1 ;
  uint8_t VARIO_MODE :1;
  uint8_t PID_MODE :1;
} flags_struct_t;

typedef struct {
  uint8_t currentSet;
  int16_t accZero[3];
  int16_t magZero[3];
  uint16_t flashsum;
  uint8_t checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
} global_conf_t;

struct pid_ {
  uint8_t P8;
  uint8_t I8;
  uint8_t D8;
};

struct servo_conf_ {  // this is a generic way to configure a servo, every multi type with a servo should use it
  int16_t min;        // minimum value, must be more than 1020 with the current implementation
  int16_t max;        // maximum value, must be less than 2000 with the current implementation
  int16_t middle;     // default should be 1500
  int8_t  rate;       // range [-100;+100] ; can be used to ajust a rate 0-100% and a direction
};

/*
 * HOLY SHIT I FOUND IT.
 * This is the configuration structure that houses all the variables we need to work with. Everything that we can change to keep it flying. - Dan
 */
typedef struct {
  pid_    pid[PIDITEMS]; //PIDITEMS is just the index of the last element of the "pid" enum (so, basically, it's the size of that structure, in the unit of "elements" (elements are (PID in front of each, omitted for space) PIDROLL, PIDPITCH, ...YAW, ALT, POS, POSR, NAVR, LEVEL, MAG, VEL, ITEMS). Means you can access that element using the name rather than the element number.
  pid_    pidset[NUMPIDSETS][PIDSETITEMS]; //NUMPIDSETS is defined as 2, so I guess there are two PID sets, and PIDSETITEMS is, similarly, the index of the last element in the "pid_set" enum
  uint8_t rcRate8; //defined and used in tuning in EEPROM.cpp (based on its value, returns a particular location for use in a lookup table) to get dynamic ROLL&PITCH PID adjustments based on RC throttle position.
  uint8_t rcExpo8; //another value defined and used in tuning in EEPROM.cpp (contributes to the lookup table location). Lookup table is implemented in MultiWii.cpp.
  uint8_t rollPitchRate; //defined in EEPROM.cpp, and used in MultiWii.cpp -- basically, it's like a HUGE rcExpo8 adjustment, designed to lessen the correction control and increase the rotation rate when the RC stick is far from centre (to make flips easier)
  uint8_t yawRate; //defined in EEPROM.cpp, used in MultiWii.cpp in the YAW PID, since it defines the rate at which the thing can turn through the vertical axis -- use it do get the error term
  uint8_t dynThrPID; //defined in EEPROM.cpp, used in MultiWii.cpp in the dynamic ROLL/PITCH PID, where it makes adjustments dynamically based on the throttle value
  uint8_t thrMid8; //defined and used in EEPROM.cpp -- along with rcRate8, rcExpo8 -- gives the location in the lookup table for mid throttle/expo values when applying "rcCommand"s
  uint8_t thrExpo8; //defined in EEPROM.cpp and used in MultiWii.cpp -- along with rcRate8, rcExpo8, and thrMid8 -- giving the location in the lookup table when appling "rcCommand"s
  int16_t angleTrim[2]; //tiny array defined in both EEPROM.cpp and Sensors.cpp and used to distinguish between roll and pitch when calculating angular error in MultiWii.cpp and Sensors.cpp; looks at stick locations to set values for angleTrim[PITCH/ROLL], then compares to saved values from previous iteration. 
  uint16_t activate[CHECKBOXITEMS]; //CHECKBOXITEMS is the index of the last element of the "box" enum, which houses a checklist of flight mode settings ("headfree", "accro", etc.)
  uint8_t powerTrigger1; //for use in alarm system (I guess for when the power level gets too low?)
  #if MAG
    int16_t mag_declination; //sets the declination, for use in magnetometer atan2 stuff to determine heading -- may need to be set for your location
  #endif
  servo_conf_ servoConf[8]; //see immediately above in types.h; generic way to configure a servo (defining min (2), max (2), middle (2), and rate(1)) for 8 servos
  #if defined(GYRO_SMOOTHING) //setting for separating and averaging gyro PITCH, YAW, ROLL -- not appropriate for multicopters, so don't fuck with this
    uint8_t Smoothing[3];
  #endif
  #if defined (FAILSAFE) //for landing when something goes wrong (see line 802 of MultiWii.cpp), set the rcData to the failsafe_throttle value and lands
    int16_t failsafe_throttle;
  #endif
  #ifdef VBAT //bunch of stuff for battery related measurements and warnings
    uint8_t vbatscale;
    uint8_t vbatlevel_warn1;
    uint8_t vbatlevel_warn2;
    uint8_t vbatlevel_crit;
  #endif
  #ifdef POWERMETER
    uint8_t pint2ma; //for calculating power, gets the scales amps
  #endif
  #ifdef POWERMETER_HARD
    uint16_t psensornull;
  #endif
  #ifdef MMGYRO
    uint8_t mmgyro;
  #endif
  #ifdef ARMEDTIMEWARNING
    uint16_t armedtimewarning;
  #endif
  int16_t minthrottle;    // for setting the minthrottle value
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} conf_t;

#ifdef LOG_PERMANENT
typedef struct {
  uint16_t arm;           // #arm events
  uint16_t disarm;        // #disarm events
  uint16_t start;         // #powercycle/reset/initialize events
  uint32_t armed_time ;   // copy of armedTime @ disarm
  uint32_t lifetime;      // sum (armed) lifetime in seconds
  uint16_t failsafe;      // #failsafe state @ disarm
  uint16_t i2c;           // #i2c errs state @ disarm
  uint8_t  running;       // toggle on arm & disarm to monitor for clean shutdown vs. powercut
  uint8_t  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE !
} plog_t;
#endif

#endif /* TYPES_H_ */
