//############################################################################################
// 
// CCSR Robot project: http://letsmakerobots.com/robot/project/ccsr
//
// Propulsion: DC motor driver for Seeedstudio Grove dual I2C DC motor driver board
//
// date: November 2014
//
//
//############################################################################################


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include "ccsr.h"
#include "motor.h"
#include "utils.h"
#include "actions.h"
#include "irSensors.h"
#include <linux/i2c-dev.h>


#ifndef I2C_SLAVE 
   #define I2C_SLAVE 0  
#endif

extern FILE *logFile;
extern int i2cbus;
extern pthread_mutex_t semI2c;
extern ccsrStateType ccsrState;

// This gets called whenever CCSR process is started
int initMotors() {
   setMotorPrescalerFrequency(1);
}

// Motor diagnostics
int motorDiagnostic() {

   logMsg(logFile, "Running motor diagnostics", LOG);

   int diagSpeed = 150;
   int turnSpeed = 230;
   int delay     = 500000;


   // Turn left/right
   while(!speedFiltered(0, turnSpeed)) {
      usleep(BRAIN_LOOP_INTERVAL);
   }
   usleep(delay);
   while(!speedFiltered(0, 0)) {
      usleep(BRAIN_LOOP_INTERVAL);
   }
   while(!speedFiltered(0, -turnSpeed)) {
      usleep(BRAIN_LOOP_INTERVAL);
   }
   usleep(delay);
   while(!speedFiltered(0, 0)) {
      usleep(BRAIN_LOOP_INTERVAL);
   }

   // Fwd/backward
   while(!speedFiltered(diagSpeed, 0)) {
      usleep(BRAIN_LOOP_INTERVAL);
   }
   usleep(delay);
   while(!speedFiltered(0, 0)) {
      usleep(BRAIN_LOOP_INTERVAL);
   }
   while(!speedFiltered(-diagSpeed, 0)) {
      usleep(BRAIN_LOOP_INTERVAL);
   }
   sleep(1);
   while(!speedFiltered(0, 0)) {
      usleep(BRAIN_LOOP_INTERVAL);
   }

   return 1;
}
 
// Set 'motor' address [MOTOR1, MOTOR2] to 'speed' [-255..255] 
// Negative motorspeed is reverse. 
// This function sets motorspeed directly, so large jumps should be avoided
// The function updates ccsrState.speedMotor[1,2] to reflect change
void setMotorspeed(int speed, unsigned char motor) {

   char buffer[4];
   int dir;

   if (speed<0) {
      dir = REVERSE;
   }
   else {
      dir = FORWARD;
   }

   buffer[0] = motor;
   buffer[1] = (char) dir;
   buffer[2] = (char) abs(speed);
   if(!ccsrState.noMotors) {

      pthread_mutex_lock(&semI2c);

      if(ioctl(i2cbus, I2C_SLAVE, MOTOR_ADDR)) {
   	 logMsg(logFile, "Can't set Motor I2C address", ERROR);
      }
      usleep(I2C_DELAY);
      #ifndef DEBUG
       if(write(i2cbus, buffer, 3) != 3) {
            logMsg(logFile, "Unsuccessful write to I2C Motor", ERROR);
         }
      #endif
      usleep(I2C_DELAY);
      pthread_mutex_unlock(&semI2c);
   }
   if(motor == MOTOR1) {
      ccsrState.speedMotor1 = speed;
   }
   else {
      ccsrState.speedMotor2 = speed;
   }
} 

// Set DC motor driver prescaler frequency
void setMotorPrescalerFrequency(int freq) {

   char buffer[4];

   buffer[0] = SETFREQ;
   buffer[1] = (char) freq;
   buffer[2] = (char) 0;
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, MOTOR_ADDR)) {
      logMsg(logFile, "Can't set Motor I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer, 3) != 3) {
      logMsg(logFile, "Unsuccessful write to I2C Motor", ERROR);       
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
} 

// Set PWM for both motors. This seems to be identical to changing MOTOR[1,2] address directly, so
// this function is unused for now.
void setMotorPWM(int speedA, int speedB) {

   char buffer[4];

   buffer[0] = SETPWMAB;
   buffer[1] = (char) speedA;
   buffer[2] = (char) speedB;
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, MOTOR_ADDR)) {
      logMsg(logFile, "Can't set Motor I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer, 3) != 3) {
      logMsg(logFile, "Unsuccessful write to I2C Motor", ERROR);       
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
} 

// Read I2C address 'reg' from DC motor driver board, return result. Function only for 
// debug purposes, not normally called.
void readMotor(int reg) {
   char buffer[4];
   char conv[2];
      
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, MOTOR_ADDR)) {
      logMsg(logFile, "Can't set ADC0 I2C address", ERROR);
   }
   usleep(I2C_DELAY);


   buffer[0] = (char) reg;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful ADC_REG_CONV write to ADC0", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 2) != 2) {
      printf("err");
      logMsg(logFile, "Unsuccessful ADC_REG_CONV read from ADC0", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

   printf("reg %d: %d %d\n", reg, conv[1], conv[0]);

} 

// This function sets CCSR forward speed and turn speed, but applies a filter, such that
// speed does not jump to target speed directly. Calling this function will only bring the
// speed one step closer to target fwd and turn speeds, and the step size is based on a 
// quantization function. The function returns 1 if target speeds are reached, 0 otherwise
// We may have to call this function multiple times with the same argument before target speeds are reached
//
// targetSpeed = [-255..255], negative is backward.
// delta = [0..511], this is the positive delta between MOTOR1 and targetSpeed, and the negative delta between
// MOTOR2 and targetSpeed, thus determining turn speed
// 
// e.g. targetSpeed = 120, delta = 10 (forward momentum with slight turn to right)
//      motor1 = 110
//      motor2 = 130
// e.g. targetSpeed = 0, delta = 100 (in-place turn around Z-axis)
//      motor1 = 100
//      motor2 = -100
int speedFiltered(int targetSpeed, int delta) {

   int diff1, diff2;
   int increment1, increment2;
   int targetReached1, targetReached2;
   int targetSpeedMotor1, targetSpeedMotor2;
   


   targetReached1 = targetReached2 = 0;
   
   targetSpeedMotor1 = targetSpeed + delta;
   targetSpeedMotor2 = targetSpeed - delta;

   diff1 = ccsrState.speedMotor1 - targetSpeedMotor1;
   diff2 = ccsrState.speedMotor2 - targetSpeedMotor2;

   // Quentization function using MOTOR_SPEEDUP_STEPS steps
   if(abs(diff1) < MAX_MOTOR_SPEED/MOTOR_SPEEDUP_STEPS) {
      increment1 = -diff1;
      targetReached1 = 1;
   }
   else if (diff1<0) {
      increment1 = MAX_MOTOR_SPEED/MOTOR_SPEEDUP_STEPS;
      targetReached1 = 0;
   }
   else {
      increment1 = -MAX_MOTOR_SPEED/MOTOR_SPEEDUP_STEPS;
      targetReached1 = 0;
   }

   if(abs(diff2) < MAX_MOTOR_SPEED/MOTOR_SPEEDUP_STEPS) {
      increment2 = -diff2;
      targetReached2 = 1;
   }
   else if (diff2<0) {
      increment2 = MAX_MOTOR_SPEED/MOTOR_SPEEDUP_STEPS;
      targetReached2 = 0;
   }
   else {
      increment2 = -MAX_MOTOR_SPEED/MOTOR_SPEEDUP_STEPS;
      targetReached2 = 0;
   }

   if (diff1 != 0) {
      setMotorspeed(ccsrState.speedMotor1 + increment1, MOTOR1);
   }
   if (diff2 != 0) {
      setMotorspeed(ccsrState.speedMotor2 + increment2, MOTOR2);
   }

#ifdef DEBUG
   if((diff1 !=0) || (diff2!=0)){
      printf("motor1: %d motor2:  %d\n", ccsrState.speedMotor1, ccsrState.speedMotor2);
}
#endif

   return targetReached1 && targetReached2;   
} 



