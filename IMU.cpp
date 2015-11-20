/*
 * IMU. Sets up the Inertial Measurement Unit (combined accelerometer and gyro sensors).
 * 
 * The basic idea is that you sense interial forces in cardinal components. By comparing
 * the relative magnitudes in each component (and comparing with gyro readings),
 * angular orientation in space (direction of g) and changes in motion can be calculated.
 * 
 * We have to include the gyro because the accelerometers measure ALL forces on the
 * system -- including the actuation that makes the thing fly.
 * 
 * Gyros are good short term (they don't get messed up by external forces) but they
 * tend to drift because they rely on integration (errors pile up). Accelerometers
 * are good for long-term stability, even though they are prone to short-term errors.
 * Hence, it's a good idea to low-pass accelerometer data.
 *
 * It uses a complementary filter, rather than a Kalman filter. It balances the long-
 * and short-term accuracy of the two sensing systems by weighting the importance of
 * each. It has the added benefit that results are easier to compute.
 * 
 * The basic filter is: angle = 0.98*(angle + gyrData*dt) + 0.02*(accelData)
 * 
 * You can see below, we compute the angle from accelerometer data using the atan2
 * function. This is standard.
 * 
 * Check it out, here: http://www.pieter-jan.com/node/11
 *
 */

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "IMU.h"
#include "Sensors.h"

void getEstimatedAttitude();

void computeIMU () {
  uint8_t axis;

    Gyro_getADC();
    ACC_getADC();
    getEstimatedAttitude();
    annexCode();
    for (axis = 0; axis < 3; axis++) {
      imu.gyroData[axis] = imu.gyroADC[axis] >> 2;
    }
}

// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
   Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
   Comment this if  you do not want filter at all.
   unit = n power of 2 */
// this one is also used for ALT HOLD calculation, should not be changed
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
  #define GYR_CMPF_FACTOR 600
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250

//****** end of advanced users settings *************
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))    //the 1.0f ensures that you're working with a literal float value (it's just a 1, though)
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

typedef struct fp_vector {		                //basically, lets you refer to "struct fp_vector" using the variable named "t_fp_vector_def", so you don't have to keep typing "struct"
  float X,Y,Z;		                            //defines what the structure (grouped list of variables referred to by the given tag) is made up of -- here, three floats
} t_fp_vector_def;                            //to access a member of the structure, you use structure_tag.name_of_variable

typedef union {		                            //a union is like a structure over a shared space in memory (shared by all the variables defined inside -- but only big enough to fit the largest member at one time)
  float A[3];		                              //note: structures must contain variables of the same type, but unions are multi-purpose (only care about size in memory, like building a chest to toss stuff in)
  t_fp_vector_def V;		                      //so here we're carving out a space the size of a 1x3 array of floats, into which we will shove the members of t_fp_vector_def
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;                           //now we have two "chests" into which we can throw our vector components: one sized for floating point input, one for 32bit ints

int16_t _atan2(int32_t y, int32_t x){         //defining the atan2 function for x-y inputs
  float z = (float)y / x;                     //casts the division of y/x as a float and stores in z
  int16_t a;                                  //perform atan2 calculation for various mixes of x,y input
  if ( abs(y) < abs(x) ){
     a = 573 * z / (1.0f + 0.28f * z * z);
   if (x<0) {
     if (y<0) a -= 1800;
     else a += 1800;
   }
  } else {
   a = 900 - 573 * z / (z * z + 0.28f);
   if (y<0) a -= 1800;
  }
  return a;
}

float InvSqrt (float x){                      //funky little algorithm for calculating inverse square roots of 32 bit numbers (see https://en.wikipedia.org/wiki/Fast_inverse_square_root), used for normalization
  union{                                      //basically, it estimates the inverse square root by subtracting from that magic number down there, then refining with a Newton's method iteration -- used to get vector magnitudes
    int32_t i;  
    float   f; 
  } conv; 
  conv.f = x; 
  conv.i = 0x5f3759df - (conv.i >> 1);        //this magic number (0x5f3759df) minus the bit pattern of x and i, shifted right one position
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);    //Newton's method on that result (treated as a floating point value) gets you much closer
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}


static int32_t accLPF32[3]    = {0, 0, 1};
static float invG; // 1/|G|

static t_fp_vector EstG;
static t_int32_t_vector EstG32;
#if MAG
  static t_int32_t_vector EstM32;
  static t_fp_vector EstM;
#else
  static t_fp_vector EstN = { 1000.0, 0.0, 0.0 };
  static t_int32_t_vector EstN32;
#endif

void getEstimatedAttitude(){
  uint8_t axis;
  int32_t accMag = 0;
  float scale, deltaGyroAngle[3];
  uint8_t validAcc;

  scale = SCALECORR * GYRO_SCALE; // GYRO_SCALE unit: radian/microsecond

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = imu.gyroADC[axis]  * scale; // radian

    accLPF32[axis]    -= accLPF32[axis]>>ACC_LPF_FACTOR;
    accLPF32[axis]    += imu.accADC[axis];
    imu.accSmooth[axis]    = accLPF32[axis]>>ACC_LPF_FACTOR;

    accMag += (int32_t)imu.accSmooth[axis]*imu.accSmooth[axis] ;
  }

  rotateV(&EstG.V,deltaGyroAngle);
  #if MAG
    rotateV(&EstM.V,deltaGyroAngle);
  #else
    rotateV(&EstN.V,deltaGyroAngle);
  #endif

  accMag = accMag*100/((int32_t)ACC_1G*ACC_1G);
  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  for (axis = 0; axis < 3; axis++) {
    if ( validAcc )
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + imu.accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float	
    #if MAG
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + imu.magADC[axis]) * INV_GYR_CMPFM_FACTOR;
      EstM32.A[axis] = EstM.A[axis];
    #else
      EstN32.A[axis] = EstN.A[axis];
    #endif
  }
  
  if ((int16_t)EstG32.A[2] > ACCZ_25deg)
    f.SMALL_ANGLES_25 = 1;
  else
    f.SMALL_ANGLES_25 = 0;

  // Attitude of the estimated vector
  int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
  invG = InvSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
  att.angle[ROLL]  = _atan2(EstG32.V.X , EstG32.V.Z);
  att.angle[PITCH] = _atan2(EstG32.V.Y , InvSqrt(sqGX_sqGZ)*sqGX_sqGZ);

  #if MAG
    att.heading = _atan2(
      EstM32.V.Z * EstG32.V.X - EstM32.V.X * EstG32.V.Z,
      (EstM.V.Y * sqGX_sqGZ  - (EstM32.V.X * EstG32.V.X + EstM32.V.Z * EstG32.V.Z) * EstG.V.Y)*invG ); 
    att.heading += conf.mag_declination; // Set from GUI
    att.heading /= 10;
  #else
    att.heading = _atan2(
      EstN32.V.Z * EstG32.V.X - EstN32.V.X * EstG32.V.Z,
      EstN32.V.Y * invG * sqGX_sqGZ  - (EstN32.V.X * EstG32.V.X + EstN32.V.Z * EstG32.V.Z) * invG * EstG32.V.Y ); 
   att.heading /= 10;
  #endif

  #if defined(THROTTLE_ANGLE_CORRECTION)
    cosZ = (EstG.V.Z / ACC_1G) * 100.0f;                                                        // cos(angleZ) * 100 
    throttleAngleCorrection = THROTTLE_ANGLE_CORRECTION * constrain(100 - cosZ, 0, 100) >>3;  // 16 bit ok: 200*150 = 30000  
  #endif
}

#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define BARO_TAB_SIZE   21

#define ACC_Z_DEADBAND (ACC_1G>>5) // was 40 instead of 32 now


#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }

#if BARO
uint8_t getEstimatedAltitude(){
  int32_t  BaroAlt;
  static float baroGroundTemperatureScale,logBaroGroundPressureSum;
  static float vel = 0.0f;
  static uint16_t previousT;
  uint16_t currentT = micros();
  uint16_t dTime;

  dTime = currentT - previousT;
  if (dTime < UPDATE_INTERVAL) return 0;
  previousT = currentT;

  if(calibratingB > 0) {
    logBaroGroundPressureSum = log(baroPressureSum);
    baroGroundTemperatureScale = (baroTemperature + 27315) *  29.271267f;
    calibratingB--;
  }

  // baroGroundPressureSum is not supposed to be 0 here
  // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
  BaroAlt = ( logBaroGroundPressureSum - log(baroPressureSum) ) * baroGroundTemperatureScale;

  alt.EstAlt = (alt.EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)

  #if (defined(VARIOMETER) && (VARIOMETER != 2)) || !defined(SUPPRESS_BARO_ALTHOLD)
    //P
    int16_t error16 = constrain(AltHold - alt.EstAlt, -300, 300);
    applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position
    BaroPID = constrain((conf.pid[PIDALT].P8 * error16 >>7), -150, +150);

    //I
    errorAltitudeI += conf.pid[PIDALT].I8 * error16 >>6;
    errorAltitudeI = constrain(errorAltitudeI,-30000,30000);
    BaroPID += errorAltitudeI>>9; //I in range +/-60
 
    // projection of ACC vector to global Z, with 1G subtructed
    // Math: accZ = A * G / |G| - 1G
    int16_t accZ = (imu.accSmooth[ROLL] * EstG32.V.X + imu.accSmooth[PITCH] * EstG32.V.Y + imu.accSmooth[YAW] * EstG32.V.Z) * invG;

    static int16_t accZoffset = 0;
    if (!f.ARMED) {
      accZoffset -= accZoffset>>3;
      accZoffset += accZ;
    }  
    accZ -= accZoffset>>3;
    applyDeadband(accZ, ACC_Z_DEADBAND);

    static int32_t lastBaroAlt;
    //int16_t baroVel = (alt.EstAlt - lastBaroAlt) * 1000000.0f / dTime;
    int16_t baroVel = (alt.EstAlt - lastBaroAlt) * (1000000 / UPDATE_INTERVAL);
    lastBaroAlt = alt.EstAlt;

    baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
    applyDeadband(baroVel, 10); // to reduce noise near zero

    // Integrator - velocity, cm/sec
    vel += accZ * ACC_VelScale * dTime;

    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity). 
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    vel = vel * 0.985f + baroVel * 0.015f;

    //D
    alt.vario = vel;
    applyDeadband(alt.vario, 5);
    BaroPID -= constrain(conf.pid[PIDALT].D8 * alt.vario >>4, -150, 150);
  #endif
  return 1;
}
#endif //BARO
