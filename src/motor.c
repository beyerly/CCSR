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
#include "servoCtrl.h"
#include "sound.h"
#include <linux/i2c-dev.h>


#ifndef I2C_SLAVE 
   #define I2C_SLAVE 0  
#endif

extern FILE *logFile;
extern int i2cbus;
extern pthread_mutex_t semI2c;
extern ccsrStateType ccsrState;
extern int pipeSoundGen[2];
extern soundType sound[standardSoundsCount];

// This gets called whenever CCSR process is started
int initMotors() {
   setMotorPrescalerFrequency(0);
   setMotorspeed(0, MOTOR1);
   setMotorspeed(0, MOTOR2);
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



// Pthread process that will continuously attempt to drive towards target heading if ccsrStte.driveToTargetHeading=1. 
// If ccstState.evasiveAction = 1, it will take evasive actions around objects encountered if necessary. To do this, it will
// turn away from the object untill a clear path is found, then push the current heading onto a 'heading-stack', and continue
// onto 'deviated' heading untill blockage is resolved. It will turn the sonar to previous heading, and scan to see if that
// direction is clear. Everytime we encounter another blockage, we will push further down the heading stack, everytime a blockage
// is resolve, we pop one level up. If we are deeper than HEADING_QUEUE_DEPTH, we give up, and assume we have been 'enclosed'
//
// It is the responsibility of other processes to 
//    - suspend by setting ccsrStte.driveToTargetHeading=0 if a target has been reached
//    - Continuously update ccstState.targetHeading if target changes.
// 
// Enables navigation, sonar and proximity sensors if nescessary
void *driveToTargetHeading() {
   char navigationOnPrev;
   char proximitySensorsOnPrev;
   char sonarSensorsOnPrev;
   char trackTargetColorOnPrev;
   char active;
   int delta;
   int headingQueue[HEADING_QUEUE_DEPTH];
   char hQPtr;

   active = 0;
   hQPtr = 0;
         // process is active

   while(1) {
      if (ccsrState.driveToTargetHeading){
         if(!active){
            // Was not active before, prepare for driving
            active = 1;
            navigationOnPrev = ccsrState.navigationOn;
            proximitySensorsOnPrev = ccsrState.proximitySensorsOn;
            // Turn on compass and proximity sensors if necessary
            ccsrState.navigationOn = 1;
            ccsrState.proximitySensorsOn = 1;
	    while(!ccsrState.proximitySensorsOn_active){
	       brainCycle();  // wait until prox sensors are active
            }
	 }
         // Calculate if we are on track
         delta = abs(ccsrState.heading - ccsrState.targetHeading);
         if(delta > 180) {
            delta = 360 - delta;
         }
         if(delta < TARGET_HEADING_HYSTERESIS) {
            // We are heading in target direction, drive ahead if possible
	    while(!speedFiltered(ccsrSpeed(), 0)) {
	       brainCycle();
	    }
            if(ccsrStateRest()) {
               // Ran into object and are stopped. 
               if(ccsrState.evasiveAction) {
                  // We do not expect this object, so do evasive action. First turn away from object
                  evasiveActionSimple();
                  // Turn target object trqacking off: we need to control the had ourselves for sonar:
                  trackTargetColorOnPrev = ccsrState.trackTargetColorOn;
                  ccsrState.trackTargetColorOn = 0;
                  // Turn Sonar on
                  sonarSensorsOnPrev = ccsrState.sonarSensorsOn;
                  ccsrState.sonarSensorsOn = 1;
                  // Turn sonar (head) back into target direction so we can track obstace at 90 deg (max for pan)
                  printf("aaa  %d %d \n", ccsrState.targetHeading, getPanToHeading(ccsrState.targetHeading));
		  setPanTilt(getPanToHeading(ccsrState.targetHeading),0,80);
                  // Push target heading into heading Q
                  headingQueue[hQPtr] = ccsrState.targetHeading;
                  hQPtr=hQPtr+1;
                  if(hQPtr>=HEADING_QUEUE_DEPTH){
                     // Heading stack overflow: we give up and go back to original target heading to try again
                     // If we are enclosed, this may be infinite loop.  
                     hQPtr=0;
                     ccsrState.targetHeading = headingQueue[hQPtr];
                  }
                  // Set target heading to current heading. So we will now continue along deviated heading untill blockage is resolved
                  ccsrState.targetHeading = ccsrState.heading;
               }
               else {
                  // We are not doing evasive action, so hitting an object means we assume we've arrived at target.
                  // Stop driving to target
                  ccsrState.driveToTargetHeading = 0;
                  ccsrState.targetReached = 1;
		  write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[singleA]));

               }
            }
            else if(hQPtr>0){
               // We are driving to target heading, but we have another target heading in the queue, 
               // which means we are on a deviated heading due to an obstacle.
               // Check if blockage is resolved, return to previous heading if so
               if (ccsrState.sonarDistFront > MIN_SONAR_DIST_FRONT_TO_DRIVE) {
                  // We have driven on deviated heading far enough, blockage is removed,  
                  // Stop
                  while(!speedFiltered(0, 0)) {
                    brainCycle();
                  }
                  // Pop previous target heading form Q
                  hQPtr = hQPtr - 1; // No clipping necessary, hQPtr is always > 0 here
                  ccsrState.targetHeading = headingQueue[hQPtr];
                  // Turn back to previous target heading
                  turnToTargetHeading(NOSCAN);
                  if(hQPtr>0) {
                     // We are not at the top of the stack yet, the current heading is another deviated heading, so turn sonar to
                     // next target heading on the stack and continue finding way around object 
                     setPanTilt(getPanToHeading(headingQueue[hQPtr-1]),0,80);
                  }
                  else{
                     // We reached top of the stack, so we are heading towards original target again: Done
                     // Pan back dead ahead
                     setPanTilt(0, 0, 80);
                     // Turn target object tracking and sonar back to original states
                     ccsrState.trackTargetColorOn = trackTargetColorOnPrev;
                     ccsrState.sonarSensorsOn = sonarSensorsOnPrev;
                  }
               }
            }
            // Check for conditions that would end diving to target operation:
            if (ccsrState.objectTracked & ccsrState.trackTargetColorOn){
               if(ccsrState.targetVisualObject_Vol > ccsrState.targetColorVolume) {
                  // Object is big enough, it must be very close, consider target is reached: stop
                  // We may need to fine-tune position later if we want to pick up tracked object with arm
                  while(!speedFiltered(0, 0)) {
                     brainCycle();
                  }
                  usleep(3000000);
                  say("Arrived at target, what can I do for you?");
                  ccsrState.driveToTargetHeading = 0;
               }
            }
         }
         else{
            // We are off-target, stop forward motion and turn back into target heading
            while(!speedFiltered(0, 0)) {
              brainCycle();
            }
            // If we are tracking object with camera, Wait until cam centers on object
            if (ccsrState.objectTracked & ccsrState.trackTargetColorOn){
               while(!ccsrState.trackedObjectCentered) {
                  brainCycle();
               }
            }
            turnToTargetHeading(NOSCAN);
            // If we are tracking object with camera, Wait until cam centers on object
            if (ccsrState.objectTracked & ccsrState.trackTargetColorOn){
               while(!ccsrState.trackedObjectCentered) {
                  brainCycle();
               }
            }
         }
      }
      // Process is not active
      if(!ccsrState.driveToTargetHeading && active) {
         // Driving to target mode was just switched off, stop forward motion and set nav and proximity back to previous states
         while(!speedFiltered(0, 0)) {
            brainCycle();
         }
         ccsrState.navigationOn       = navigationOnPrev;
         ccsrState.proximitySensorsOn = proximitySensorsOnPrev;
         active = 0;
      }
      // Process is not active, and was not just switched off either (!driveToTargetHeading || !active):
      // Do nothing until driveToTargetHeading is turned on
      usleep(BRAIN_LOOP_INTERVAL);
   }
}
