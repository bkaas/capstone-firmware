#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"

void initializeSoftPWM(void);

#if defined(SERVO)
void initializeServo();
#endif

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
#if defined(PROMINI)
  uint8_t PWM_PIN[8] = {9,10,11,3,6,5,A2,12};   //for a quad+: rear,right,left,front
#endif
#if defined(PROMICRO)
  uint8_t PWM_PIN[8] = {9,10,5,6,11,13};   //for a quad+: rear,right,left,front
#endif
#if defined(MEGA)
  uint8_t PWM_PIN[8] = {3,5,6,2,7,8,9,10};      //for a quad+: rear,right,left,front   //+ for y6: 7:under right  8:under left
#endif

/**************************************************************************************/
/***************   Writes the Servos values to the needed format   ********************/
/**************************************************************************************/
void writeServos() {
  #if defined(SERVO)
    // write HW PWM servos for the mega
    #if defined(MEGA) && defined(MEGA_HW_PWM_SERVOS)
      #if (PRI_SERVO_FROM == 1 || SEC_SERVO_FROM == 1) 
        OCR5C = servo[0];
      #endif 
      #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
        OCR5B = servo[1];
      #endif 
      #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
        OCR5A = servo[2];
      #endif 
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
        OCR1A = servo[3];
      #endif 
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
        OCR1B = servo[4];
      #endif 
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
        OCR4A = servo[5];
      #endif 
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
        OCR4B = servo[6];
      #endif 
      #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
        OCR4C = servo[7];
      #endif
    #endif
    // write HW PWM servos for the promicro
    #if defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS)
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7)
        OCR1A = servo[6];// Pin 9
      #endif
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5)
        OCR1B = servo[4];// Pin 10
      #endif
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6)
        OCR3A = servo[5];// Pin 5
      #endif
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4)
        OCR1C = servo[3];// Pin 11
      #endif
    #endif
  #endif
}

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]
  #if !(NUMBER_MOTOR == 4) 
    #error "only 4 motors allowed"
  #endif
  
  #if defined(MEGA)
    #error "only arduino 328 or 32u4"
  #endif

  #if defined(PROMINI)
    #if defined(EXT_MOTOR_32KHZ)
      OCR1A = (motor[0] - 1000) >> 2; //  pin 9
      OCR1B = (motor[1] - 1000) >> 2; //  pin 10
      OCR2A = (motor[2] - 1000) >> 2; //  pin 11
      OCR2B = (motor[3] - 1000) >> 2; //  pin 3
    #elif defined(EXT_MOTOR_4KHZ)
      OCR1A = (motor[0] - 1000) << 1; //  pin 9
      OCR1B = (motor[1] - 1000) << 1; //  pin 10
      OCR2A = (motor[2] - 1000) >> 2; //  pin 11
      OCR2B = (motor[3] - 1000) >> 2; //  pin 3
    #elif defined(EXT_MOTOR_1KHZ)
      OCR1A = (motor[0] - 1000) << 3; //  pin 9
      OCR1B = (motor[1] - 1000) << 3; //  pin 10
      OCR2A = (motor[2] - 1000) >> 2; //  pin 11
      OCR2B = (motor[3] - 1000) >> 2; //  pin 3
    #else
      #error only 32khz or 4khz or 1khz on 328 device
    #endif    
  #endif
  
  #if defined(PROMICRO)
    uint16_t Temp2;
    Temp2 = motor[3] - 1000;
    #if defined(EXT_MOTOR_64KHZ)
      OCR1A = (motor[0] - 1000) >> 2; // max = 255
      OCR1B = (motor[1] - 1000) >> 2;
      OCR3A = (motor[2] - 1000) >> 2;
      Temp2 = Temp2 >> 2;
      TC4H = Temp2 >> 8;
      OCR4D = Temp2 & 0xFF;           //  pin 6
    #elif defined(EXT_MOTOR_32KHZ)
      OCR1A = (motor[0] - 1000) >> 1; // max = 511
      OCR1B = (motor[1] - 1000) >> 1;
      OCR3A = (motor[2] - 1000) >> 1;
      Temp2 = Temp2 >> 1;
      TC4H = Temp2 >> 8;
      OCR4D = Temp2 & 0xFF;           //  pin 6
    #elif defined(EXT_MOTOR_16KHZ)
      OCR1A = motor[0] - 1000;        //  pin 9
      OCR1B = motor[1] - 1000;        //  pin 10
      OCR3A = motor[2] - 1000;        //  pin 5
      TC4H = Temp2 >> 8;
      OCR4D = Temp2 & 0xFF;           //  pin 6
    #elif defined(EXT_MOTOR_8KHZ)
      OCR1A = (motor[0]-1000) << 1;   //  pin 9
      OCR1B = (motor[1]-1000) << 1;   //  pin 10
      OCR3A = (motor[2]-1000) << 1;   //  pin 5
      TC4H = Temp2 >> 8;
      OCR4D = Temp2 & 0xFF;           //  pin 6
    #else
      #error only 32khz or 16khz or 8khz on 32u4 device
    #endif
  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
  writeMotors();
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
    
  #if defined(PROMINI)
    TCCR1A = (1<<WGM11); // phase correct mode & no prescaler
    TCCR1B = (1<<WGM13) | (1<<CS10);
    #if defined(EXT_MOTOR_32KHZ)
      ICR1   = 0x00FF; // TOP to 255;
      TCCR2B =  (1<<CS20);
    #elif defined(EXT_MOTOR_4KHZ)
      ICR1   = 0x07F8; // TOP to 1023;     
      TCCR2B =  (1<<CS21);
    #elif defined(EXT_MOTOR_1KHZ)
      ICR1   = 0x1FE0; // TOP to 8184;     
      TCCR2B =  (1<<CS20) | (1<<CS21);
    #else
      #error only 32khz or 4khz or 1khz on 328 device
    #endif    
    TCCR1A |= _BV(COM1A1); // connect pin  9 to timer 1 channel A
    TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    TCCR2A |= _BV(COM2A1); // connect pin 11 to timer 2 channel A
    TCCR2A |= _BV(COM2B1); // connect pin  3 to timer 2 channel B
  #endif

  #if defined(PROMICRO)
    TCCR1A = (1<<WGM11);
    TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
  
    TCCR3A = (1<<WGM31);
    TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS30);
  
    #if defined(EXT_MOTOR_64KHZ)
      ICR1   = 0x00FF; // TOP to 255;
      ICR3   = 0x00FF; // TOP to 255;
      TC4H = 0x00;
      OCR4C = 0xFF; // phase and frequency correct mode & top to 255
      TCCR4B = (1<<CS40);             // prescaler to 1
    #elif defined(EXT_MOTOR_32KHZ)
      ICR1   = 0x01FF; // TOP to 511;
      ICR3   = 0x01FF; // TOP to 511;
      TC4H = 0x01;
      OCR4C = 0xFF; // phase and frequency correct mode & top to 511
      TCCR4B = (1<<CS40);             // prescaler to 1
    #elif defined(EXT_MOTOR_16KHZ)
      ICR1   = 0x03FF; // TOP to 1023;
      ICR3   = 0x03FF; // TOP to 1023;
      TC4H = 0x03;
      OCR4C = 0xFF; // phase and frequency correct mode & top to 1023
      TCCR4B = (1<<CS40);             // prescaler to 1
    #elif defined(EXT_MOTOR_8KHZ)
      ICR1   = 0x07FF; // TOP to 2046;
      ICR3   = 0x07FF; // TOP to 2046;
      TC4H = 0x3;
      OCR4C = 0xFF; // phase and frequency correct mode
      TCCR4B = (1<<CS41);             // prescaler to 2
    #else
      #error only 64khz to 8khz on 32u4 device
    #endif
  
    TCCR1A |= _BV(COM1A1); // connect pin 9 to timer 1 channel A
    TCCR1A |= _BV(COM1B1); // connect pin 10 to timer 1 channel B
    TCCR3A |= _BV(COM3A1); // connect pin 5 to timer 3 channel A
  
    TCCR4D = 0;
    TCCR4C |= (1<<COM4D1)|(1<<PWM4D); // connect pin 6 to timer 4 channel D
  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
  
  #if defined(SERVO)
    initializeServo();
  #endif
}


#if defined(SERVO)
/**************************************************************************************/
/************                Initialize the PWM Servos               ******************/
/**************************************************************************************/
void initializeServo() {
  #if defined(MEGA) && defined(MEGA_HW_PWM_SERVOS)
    #if defined(SERVO_RFR_RATE)
      #if (SERVO_RFR_RATE < 20)
        #define SERVO_RFR_RATE 20
      #endif
      #if (SERVO_RFR_RATE > 400)
        #define SERVO_RFR_RATE 400
      #endif
    #else
      #if defined(SERVO_RFR_50HZ)
        #define SERVO_RFR_RATE 50
      #elif defined(SERVO_RFR_160HZ)
        #define SERVO_RFR_RATE 160
      #elif defined(SERVO_RFR_300HZ)
        #define SERVO_RFR_RATE 300
      #endif
    #endif  
    #define SERVO_TOP_VAL (uint16_t)(1000000L / SERVO_RFR_RATE)
    // init Timer 5, 1 and 4 of the mega for hw PWM
    TIMSK5 &= ~(1<<OCIE5A); // Disable software PWM  
    #if (PRI_SERVO_TO >= 1) || (SEC_SERVO_TO >= 1)
      TCCR5A |= (1<<WGM51);   // phase correct mode & prescaler to 8 = 1us resolution
      TCCR5A &= ~(1<<WGM50);
      TCCR5B &= ~(1<<WGM52) &  ~(1<<CS50) & ~(1<<CS52);
      TCCR5B |= (1<<WGM53) | (1<<CS51);
      ICR5 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM == 1 || SEC_SERVO_FROM == 1) 
        pinMode(44,OUTPUT);
        TCCR5A |= (1<<COM5C1); // pin 44
      #endif
      #if (PRI_SERVO_FROM <= 2 && PRI_SERVO_TO >= 2) || (SEC_SERVO_FROM <= 2 && SEC_SERVO_TO >= 2) 
        pinMode(45,OUTPUT);
        TCCR5A |= (1<<COM5B1); // pin 45
      #endif
      #if (PRI_SERVO_FROM <= 3 && PRI_SERVO_TO >= 3) || (SEC_SERVO_FROM <= 3 && SEC_SERVO_TO >= 3) 
        pinMode(46,OUTPUT);
        TCCR5A |= (1<<COM5A1); // pin 46
      #endif
    #endif
    #if (PRI_SERVO_TO >= 4) || (SEC_SERVO_TO >= 4) 
      TCCR1A |= (1<<WGM11); // phase correct mode & prescaler to 8
      TCCR1A &= ~(1<<WGM10);
      TCCR1B &= ~(1<<WGM12) &  ~(1<<CS10) & ~(1<<CS12);
      TCCR1B |= (1<<WGM13) | (1<<CS11);
      ICR1 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM <= 4 && PRI_SERVO_TO >= 4) || (SEC_SERVO_FROM <= 4 && SEC_SERVO_TO >= 4) 
        pinMode(11, OUTPUT);
        TCCR1A |= (1<<COM1A1); // pin 11
      #endif
      #if (PRI_SERVO_FROM <= 5 && PRI_SERVO_TO >= 5) || (SEC_SERVO_FROM <= 5 && SEC_SERVO_TO >= 5) 
        pinMode(12,OUTPUT);
        TCCR1A |= (1<<COM1B1); // pin 12
      #endif
    #endif
    #if (PRI_SERVO_TO >= 6) || (SEC_SERVO_TO >= 6) 
      // init 16bit timer 4
      TCCR4A |= (1<<WGM41); // phase correct mode
      TCCR4A &= ~(1<<WGM40);
      TCCR4B &= ~(1<<WGM42) &  ~(1<<CS40) & ~(1<<CS42);
      TCCR4B |= (1<<WGM43) | (1<<CS41);
      ICR4 = SERVO_TOP_VAL;
      #if (PRI_SERVO_FROM <= 6 && PRI_SERVO_TO >= 6) || (SEC_SERVO_FROM <= 6 && SEC_SERVO_TO >= 6) 
        pinMode(6,OUTPUT);
        TCCR4A |= _BV(COM4A1); // connect pin 6 to timer 4 channel A
      #endif
      #if (PRI_SERVO_FROM <= 7 && PRI_SERVO_TO >= 7) || (SEC_SERVO_FROM <= 7 && SEC_SERVO_TO >= 7) 
        pinMode(7,OUTPUT);
        TCCR4A |= _BV(COM4B1); // connect pin 7 to timer 4 channel B
      #endif
      #if (PRI_SERVO_FROM <= 8 && PRI_SERVO_TO >= 8) || (SEC_SERVO_FROM <= 8 && SEC_SERVO_TO >= 8) 
        #if defined(AIRPLANE) || defined(HELICOPTER)
          servo[7] =  MINCOMMAND;    // Trhottle at minimum for airplane and heli
          OCR4C = MINCOMMAND;
        #endif  
        pinMode(8,OUTPUT);
        TCCR4A |= _BV(COM4C1); // connect pin 8 to timer 4 channel C
      #endif
    #endif 
  #endif // mega hw pwm

  #if defined(PROMICRO) && defined(A32U4_4_HW_PWM_SERVOS)
    // atm. always initialize 4 servos to pins 9, 10, 11, 5
    TIMSK1 &= ~(1<<OCIE1A) & ~(1<<OCIE1B) & ~(1<<OCIE1C);
    TCCR1A |= (1<<WGM11); // phase correct mode & prescaler to 8
    TCCR1A &= ~(1<<WGM10);
    TCCR1B &= ~(1<<WGM12) &  ~(1<<CS10) & ~(1<<CS12);
    TCCR1B |= (1<<WGM13) | (1<<CS11);
    pinMode(9,OUTPUT);
    TCCR1A |= (1<<COM1A1); // pin 9
    pinMode(10,OUTPUT);
    TCCR1A |= (1<<COM1B1); // pin 10
    pinMode(11,OUTPUT);
    TCCR1A |= (1<<COM1C1); // pin 11

    TCCR3A |= (1<<WGM31); // phase correct mode & prescaler to 8
    TCCR3A &= ~(1<<WGM30);
    TCCR3B &= ~(1<<WGM32) &  ~(1<<CS30) & ~(1<<CS32);
    TCCR3B |= (1<<WGM33) | (1<<CS31);
    pinMode(5,OUTPUT);
    TCCR3A |= (1<<COM3A1); // pin 5
    #if defined(SERVO_RFR_RATE)
      #if (SERVO_RFR_RATE < 50) || (SERVO_RFR_RATE > 400)
        #error "* invalid SERVO_RFR_RATE specified"
      #endif
      #define SERVO_TOP_VAL (uint16_t)(1000000L / SERVO_RFR_RATE)
    #elif defined(SERVO_RFR_50HZ)
      #define SERVO_TOP_VAL 16700
    #elif defined(SERVO_RFR_160HZ)
      #define SERVO_TOP_VAL 6200
    #elif defined(SERVO_RFR_300HZ)
      #define SERVO_TOP_VAL 3300
    #else
      #error "* must set SERVO_RFR_RATE or one of the fixed refresh rates of 50, 160 or 300 Hz"
    #endif
    #if defined(SERVO_PIN5_RFR_RATE)
      #if (SERVO_PIN5_RFR_RATE < 50) || (SERVO_PIN5_RFR_RATE > 400)
        #error "* invalid SERVO_PIN5_RFR_RATE specified"
      #endif
      #define SERVO_PIN5_TOP_VAL (uint16_t)(1000000L / SERVO_PIN5_RFR_RATE)
    #else
      #define SERVO_PIN5_TOP_VAL SERVO_TOP_VAL
    #endif
    ICR1   = SERVO_TOP_VAL;      // set TOP timer 1
    ICR3   = SERVO_PIN5_TOP_VAL; // set TOP timer 3
  #endif // promicro hw pwm
}
#endif


/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/

// get servo middle point from Config or from RC-Data
int16_t get_middle(uint8_t nr) {
  return (conf.servoConf[nr].middle < RC_CHANS) ? rcData[conf.servoConf[nr].middle] : conf.servoConf[nr].middle;
}

// int8_t servodir(uint8_t n, uint8_t b) { return ((conf.servoConf[n].rate & b) ? -1 : 1) ; }

void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #if defined(DYNBALANCE)
    return;
  #endif
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z
  #define SERVODIR(n,b) ((conf.servoConf[n].rate & b) ? -1 : 1)

  /****************                   main Mix Table                ******************/
  #if defined( MY_PRIVATE_MIXING )
    #include MY_PRIVATE_MIXING
  #elif defined( TRI )
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = (SERVODIR(5, 1) * axisPID[YAW]) + get_middle(5); //REAR
  #elif defined( QUADP )
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #elif defined( QUADX )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #elif defined( Y4 )
    motor[0] = PIDMIX(+0,+1,-1);   //REAR_1 CW
    motor[1] = PIDMIX(-1,-1, 0); //FRONT_R CCW
    motor[2] = PIDMIX(+0,+1,+1);   //REAR_2 CCW
    motor[3] = PIDMIX(+1,-1, 0); //FRONT_L CW
  #elif defined( VTAIL4 )
    motor[0] = PIDMIX(+0,+1, +1); //REAR_R
    motor[1] = PIDMIX(-1, -1, +0); //FRONT_R
    motor[2] = PIDMIX(+0,+1, -1); //REAR_L
    motor[3] = PIDMIX(+1, -1, -0); //FRONT_L
  #elif defined( GIMBAL )
    for(i=0;i<2;i++) {
      servo[i]  = ((int32_t)conf.servoConf[i].rate * att.angle[1-i]) /50L;
      servo[i] += get_middle(i);
    }
  #else
    #error "missing coptertype mixtable entry. Either you forgot to define a copter type or the mixing table is lacking neccessary code"
  #endif // MY_PRIVATE_MIXING

  /************************************************************************************************************/
  /****************************                Cam stabilize Servos             *******************************/

  #if defined(SERVO_TILT)
    servo[0] = get_middle(0);
    servo[1] = get_middle(1);
    if (rcOptions[BOXCAMSTAB]) {
      servo[0] += ((int32_t)conf.servoConf[0].rate * att.angle[PITCH]) /50L;
      servo[1] += ((int32_t)conf.servoConf[1].rate * att.angle[ROLL])  /50L;
    }
  #endif

  #ifdef SERVO_MIX_TILT
    int16_t angleP = get_middle(0) - MIDRC;
    int16_t angleR = get_middle(1) - MIDRC;
    if (rcOptions[BOXCAMSTAB]) {
      angleP += ((int32_t)conf.servoConf[0].rate * att.angle[PITCH]) /50L;
      angleR += ((int32_t)conf.servoConf[1].rate * att.angle[ROLL])  /50L;
    }
    servo[0] = MIDRC+angleP-angleR;
    servo[1] = MIDRC-angleP-angleR;
  #endif

/****************                    Cam trigger Servo                ******************/
  #if defined(CAMTRIG)
    // setup MIDDLE for using as camtrig interval (in msec) or RC channel pointer for interval control
    #define CAM_TIME_LOW  conf.servoConf[2].middle
    static uint8_t camCycle = 0;
    static uint8_t camState = 0;
    static uint32_t camTime = 0;
    static uint32_t ctLow;
    if (camCycle==1) {
      if (camState == 0) {
        camState = 1;
        camTime = millis();
      } else if (camState == 1) {
        if ( (millis() - camTime) > CAM_TIME_HIGH ) {
          camState = 2;
          camTime = millis();
          if(CAM_TIME_LOW < RC_CHANS) {
            ctLow = constrain((rcData[CAM_TIME_LOW]-1000)/4, 30, 250);
            ctLow *= ctLow;
          } else ctLow = CAM_TIME_LOW;
        }
      } else { //camState ==2
        if (((millis() - camTime) > ctLow) || !rcOptions[BOXCAMTRIG] ) {
          camState = 0;
          camCycle = 0;
        }
      }
    }
    if (rcOptions[BOXCAMTRIG]) camCycle=1;
    servo[2] =(camState==1) ? conf.servoConf[2].max : conf.servoConf[2].min;
    servo[2] = (servo[2]-1500)*SERVODIR(2,1)+1500;
  #endif

/************************************************************************************************************/
  // add midpoint offset, then scale and limit servo outputs - except SERVO8 used commonly as Moror output
  // don't add offset for camtrig servo (SERVO3)
  #if defined(SERVO)
    for(i=SERVO_START-1; i<SERVO_END; i++) {
      if(i < 2) {
        servo[i] = map(servo[i], 1020,2000, conf.servoConf[i].min, conf.servoConf[i].max);   // servo travel scaling, only for gimbal servos
      }
    #if defined(HELICOPTER) && (YAWMOTOR)
      if(i != 5) // not limit YawMotor
    #endif
        servo[i] = constrain(servo[i], conf.servoConf[i].min, conf.servoConf[i].max); // limit the values
    }
    #if defined(A0_A1_PIN_HEX) && (NUMBER_MOTOR == 6) && defined(PROMINI)
      servo[3] = servo[0];    // copy CamPitch value to propper output servo for A0_A1_PIN_HEX
      servo[4] = servo[1];    // copy CamRoll  value to propper output servo for A0_A1_PIN_HEX
    #endif
    #if defined(TRI) && defined(MEGA_HW_PWM_SERVOS) && defined(MEGA)
      servo[5] = constrain(servo[5], conf.servoConf[5].min, conf.servoConf[5].max); // servo[5] is still use by gui for this config (more genereic)
      servo[3] = servo[5];    // copy TRI serwo value to propper output servo for MEGA_HW_PWM_SERVOS
    #endif
  #endif

  /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
      #ifndef MOTOR_STOP
        motor[i] = conf.minthrottle;
      #else
        motor[i] = MINCOMMAND;
      #endif
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }

  /****************                      Powermeter Log                    ******************/
  #if (LOG_VALUES >= 3) || defined(POWERMETER_SOFT)
  {
    static uint32_t lastRead = currentTime;
    uint16_t amp;
    uint32_t ampsum, ampus; // pseudo ampere * microseconds
    /* true cubic function;
     * when divided by vbat_max=126 (12.6V) for 3 cell battery this gives maximum value of ~ 500
     * when divided by no_vbat=60 (6V) for 3 cell battery this gives maximum value of ~ 1000
     * */

    static uint16_t amperes[64] =   {   0,  2,  6, 15, 30, 52, 82,123,
                                     175,240,320,415,528,659,811,984,
                                     1181,1402,1648,1923,2226,2559,2924,3322,
                                     3755,4224,4730,5276,5861,6489,7160,7875,
                                     8637 ,9446 ,10304,11213,12173,13187,14256,15381,
                                     16564,17805,19108,20472,21900,23392,24951,26578,
                                     28274,30041,31879,33792,35779,37843,39984,42205,
                                     44507,46890,49358,51910,54549,57276,60093,63000};
  
    if (analog.vbat > NO_VBAT) { // by all means - must avoid division by zero
      ampsum = 0;
      for (i =0;i<NUMBER_MOTOR;i++) {
        amp = amperes[ ((motor[i] - 1000)>>4) ] / analog.vbat; // range mapped from [1000:2000] => [0:1000]; then break that up into 64 ranges; lookup amp
        ampus = ( (currentTime-lastRead) * (uint32_t)amp * (uint32_t)conf.pint2ma ) / PLEVELDIVSOFT;
        #if (LOG_VALUES >= 3)
          pMeter[i]+= ampus; // sum up over time the mapped ESC input
        #endif
        #if defined(POWERMETER_SOFT)
          ampsum += ampus; // total sum over all motors
        #endif
      }
      #if defined(POWERMETER_SOFT)
        pMeter[PMOTOR_SUM]+= ampsum / NUMBER_MOTOR; // total sum over all motors
      #endif
    }
    lastRead = currentTime;
  }
  #endif
}

/********************************************************************/
/****                         LED Handling                       ****/
/********************************************************************/
//Beware!! Is working with delays, do not use inflight!

void blinkLED(uint8_t num, uint8_t ontime,uint8_t repeat) {
  uint8_t i,r;
  for (r=0;r<repeat;r++) {
    for(i=0;i<num;i++) {
      #if defined(LED_FLASHER)
        switch_led_flasher(1);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(1);
      #endif
      LEDPIN_TOGGLE; // switch LEDPIN state
      delay(ontime);
      #if defined(LED_FLASHER)
        switch_led_flasher(0);
      #endif
      #if defined(LANDING_LIGHTS_DDR)
        switch_landing_lights(0);
      #endif
    }
    delay(60); //wait 60 ms
  }
}

