#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include "ccsr.h"
#include "sound.h"
#include "motor.h"
#include "actions.h"
#include "utils.h"
#include "irSensors.h"
#include "lcdDisp.h"
#include "powerMonitor.h"
#include "servoCtrl.h"
#include <linux/i2c-dev.h>


#ifndef I2C_SLAVE 
   #define I2C_SLAVE 0  
#endif

extern FILE *logFile;
extern int i2cbus;
extern pthread_mutex_t semI2c;
extern ccsrStateType ccsrState;
extern int  pipeLCDMsg[2];
extern soundType sound[standardSoundsCount];
extern int pipeSoundGen[2];

char lcdEvent;

// Static list of known colors by HSV value range
colorType colors[NUM_COLORS];

// Populate list of known colors by HSV range. This is obviously an approximation, we'll simply
// call all HSV values representing any shade of green "green". More detail can be added. 
// Note: Is there a HSV2string function? string s = Color.FromArgb(255, 143, 143, 143).Name seems to be
// from .NET only.
void initColors(){

   colors[0].iLowH  = 100;
   colors[0].iHighH = 120;
   colors[0].iLowS  = 140;
   colors[0].iHighS = 208;
   colors[0].iLowV  = 89;
   colors[0].iHighV = 223;
   strcpy(colors[0].name, "blue");
}

// Return pointer string representing approximate color name of HSV value. Return 0 if no match found
char* lookupColor(int H, int S, int V){

  int i;
  
  for (i=0;i<NUM_COLORS;i++) {
    if((H>=colors[i].iLowH)  &&
       (H<=colors[i].iHighH) &&
       (S>=colors[i].iLowS)  &&
       (S<=colors[i].iHighS) &&
       (V>=colors[i].iLowV)  &&
       (V<=colors[i].iHighV)){
       return colors[i].name;
    }
  }
  return 0;
}


int evasiveAction() {
   int motorSpeed, motorSpeedDelta;
   int heading_orig;
   int target_heading_orig;
   int heading_deviation;
   int heading_compensation;
   int x; 


   ccsrState.stress =  ccsrState.stress + 1;  // limit this?

   heading_orig = ccsrState.heading;
   target_heading_orig = ccsrState.targetHeading;
   ccsrState.action = EVASIVE_ACTION;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
   write(pipeSoundGen[IN], &sound[evasiveActionSnd], sizeof(sound[evasiveActionSnd]));
   
   motorSpeed = 0;

//   if((ccsrState.irDistBelow < IR_DIST_BOTTOM_THRESHOLD) ||
//      (ccsrState.currentLimit >= MAX_CURRENTLIMIT_EVENTS)) {
/*     if(ccsrState.currentLimit) {
      // We have a precipice in front, or we're stuck (stationary DC motor current), back up and turn away 90 deg

      heading_compensation = ccsrState.heading + 90;
      if( heading_compensation>360) {
     	 heading_compensation = heading_compensation - 360;
      }


      for(x=0;x<BACK_UP_LENGTH;x++) { 
   	  speedFiltered(-200, 0);
         brainCycle();
      }     
      while(!speedFiltered(0, 0)) {
     	 brainCycle();
      }
      ccsrState.targetHeading = ccsrState.heading - 90;
      if(ccsrState.targetHeading<0) {
         ccsrState.targetHeading = ccsrState.targetHeading + 360;
      }
      turnToTargetHeading(NOSCAN);
   }
*/
     if(ccsrState.currentLimit) {
      // motor stuck
      ccsrState.currentLimit = 0;
      say("Tracks stuck, backing up");
      heading_compensation = ccsrState.heading + 90;
      if( heading_compensation>360) {
     	 heading_compensation = heading_compensation - 360;
      }


      for(x=0;x<BACK_UP_LENGTH;x++) { 
   	 if(ccsrState.currentLimit) {
	    break;
	 }
	 speedFiltered(-200, 0);
         brainCycle();
      }     
      while(!speedFiltered(0, 0)) {
     	 brainCycle();
      }
      ccsrState.currentLimit = 0;
      ccsrState.targetHeading = ccsrState.heading - 90;
      if(ccsrState.targetHeading<0) {
         ccsrState.targetHeading = ccsrState.targetHeading + 360;
      }
//      turnToTargetHeading(NOSCAN);
    ccsrState.targetHeading = target_heading_orig;

   }
   else {
      // We have obstacle in front
      // Turn away from obstacle, untill obstacle is no longer visible. Turn direction is toward the ir sensor with the
      // largest distance. Return the 2x the deviated heading, which can be used to compensate for deviation from original
      // course
      if (ccsrState.irDistFrontLeft > ccsrState.irDistFrontRight) {
     	 motorSpeedDelta = MAX_MOTOR_TURNSPEED;
      }
      else {
     	 motorSpeedDelta = -MAX_MOTOR_TURNSPEED;
      }
 
      while ( max(ccsrState.irDistFrontLeft, ccsrState.irDistFrontRight) > MAX_IR_DIST_TO_DRIVE) {
         speedFiltered(motorSpeed, motorSpeedDelta);
     	 brainCycle();
//	 printf("ww %d %d\n", ccsrState.irDistFrontLeft, ccsrState.irDistFrontRight);
      }
      while(!speedFiltered(0, 0)) {
     	 brainCycle();
      }

      heading_deviation = ccsrState.heading;
      heading_compensation = 2*heading_orig - heading_deviation;
      if( heading_compensation<0) {
     	 heading_compensation = heading_compensation + 360;
      }
      if( heading_compensation>360) {
     	 heading_compensation = heading_compensation - 360;
      }
      ccsrState.action = NO_ACTION;
      lcdEvent = EVENT_ACTION;
      write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
   } 

   ccsrState.action = NO_ACTION;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
 
   return heading_compensation;
}



// Turn directly to heading in specific direction (left/right).
void turnToTargetHeadingDirect(int scan, int turnDir) {
   int motorSpeed;
   int mult; 
   int lightSensHeading;
   
   int x;

   ccsrState.action = TURN_TO_TARGET_HEADING;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
   
   motorSpeed = 0;

   if(scan) {
      // Clear ambient light and sonard distance profiles.
      for(x=0; x<=360; x++) {
   	 ccsrState.profileValid[x] = 0;
      }
   }
    
   if (turnDir==RIGHT) {
      mult = 1;
   }
   else {
      mult = -1;
   }
   printf("turn to heading %d dir: %d delta %d from %d\n", ccsrState.targetHeading, turnDir, headingDelta(ccsrState.targetHeading, turnDir), ccsrState.heading);

   // Start turn
   while(headingDelta(ccsrState.targetHeading, turnDir) > 5)  {
      if(speedFiltered(motorSpeed, mult*ccsrSpeedDelta(ccsrState.targetHeading)) && ccsrState.currentLimit){
         evasiveAction();
      }
      if(scan) {
   	 lightSensHeading = ccsrState.heading - 90;
   	 if(lightSensHeading<0) {
   	    lightSensHeading = lightSensHeading + 360;
   	 }
   	 ccsrState.ambientLightProfile[lightSensHeading] = ccsrState.ambientLight;
   	 ccsrState.sonarDistProfile[ccsrState.heading] = ccsrState.sonarDistFront;
   	 ccsrState.profileValid[ccsrState.heading] = 1;
      }

      brainCycle();
   }
   // Stop turn
   while(!speedFiltered(0, 0)) {
      brainCycle();
   }
   ccsrState.action = NO_ACTION;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
} 

// Turn shortest distance to heading.
void turnToTargetHeading(int scan) {
   int motorSpeed;
   char turnDir; 
   int lightSensHeading;
   char navigationOnPrev;  
   int x;

   navigationOnPrev = ccsrState.navigationOn;

   ccsrState.navigationOn = 1;
   turnDir = shortestTurnDir(ccsrState.targetHeading);

   turnToTargetHeadingDirect(scan, turnDir);
   ccsrState.navigationOn = navigationOnPrev;

} 



int sonarScan(int range) {

   int i, start, end;
   char sonarSensorOn_prev;
   char navigationOn_prev;
   int heading;
   
   sonarSensorOn_prev = ccsrState.sonarSensorsOn;
   navigationOn_prev = ccsrState.navigationOn;
   ccsrState.sonarSensorsOn = 1;
   ccsrState.navigationOn   = 1;
   heading = ccsrState.heading;
   
   if(range>180) {
      range = 180;
   }

   start = -range/2;
   end   =  range/2;
   

   setPanTilt(start, 0, 70);
   usleep(1000000);
   for(i=start;i<=end;i++) {
      setPanTilt(i, 0, 100);
      usleep(SONAR_SCAN_DELAY);
      ccsrState.sonarDistProfile[addAngle(i, heading)] = ccsrState.sonarDistFront;
      ccsrState.profileValid[addAngle(i, heading)] = 1;
      if(ccsrState.objectTracked) {
         ccsrState.trackTargetColorOn = 1;
         ccsrState.sonarSensorsOn = sonarSensorOn_prev;
         ccsrState.navigationOn = navigationOn_prev;
	 return 1;
      }
   }
   setPanTilt(0, 0, 70);
   ccsrState.sonarSensorsOn = sonarSensorOn_prev;
   ccsrState.navigationOn = navigationOn_prev;

   return 0;
}

void sonarScanDown() {

   char sonarSensorOn_prev;
   
   sonarSensorOn_prev = ccsrState.sonarSensorsOn;
   ccsrState.sonarSensorsOn = 1;
   
   setPanTilt(0, -40, 70);
   usleep(500000);
   ccsrState.sonarDistDownFront = ccsrState.sonarDistFront;
   setPanTilt(0, 0, 70);
   ccsrState.sonarSensorsOn = sonarSensorOn_prev;

}


// Sweep a range of 160 degrees in front, capture sonar distance and ambient light. 
// If mode = FULL, turn a full 360 degress in 3 120 degree steps, and do same 160 degress sweep each time
// If visual target object is detected by camera, turn on camera tracking, and abort sweep, return 1
// Otherwise, complete sweep, and return 0.
int orientation(int mode) {

   int x, heading_orig;

   char navigationOn_prev;

   navigationOn_prev = ccsrState.navigationOn;
   usleep(100000);

   // Turn on navigation for compass
   ccsrState.navigationOn   = 1;
   // Capture current heading
   heading_orig = ccsrState.heading;
   ccsrState.navigationOn = navigationOn_prev;

   // Clear ambient light and sonard distance profiles.
   for(x=0; x<=360; x++) {
      ccsrState.profileValid[x] = 0;
   }

   ccsrState.action = ORIENTATION;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
   write(pipeSoundGen[IN], &sound[orientationSnd], sizeof(sound[orientationSnd]));
   
   // Always do first sweep
   if(sonarScan(160)) {
      return 1;
   }
   sonarScanDown();    
   
   if(mode==FULL) {
      // DO full 360 sweep
      ccsrState.targetHeading = addAngleToHeading(120);
      turnToTargetHeading(NOSCAN);
      if(sonarScan(160)) {
         return 1;
      }
      ccsrState.targetHeading = addAngleToHeading(120);
      turnToTargetHeading(NOSCAN);
      if(sonarScan(160)) {
         return 1;
      }
   }


   write(pipeSoundGen[IN], &sound[orientationSnd], sizeof(sound[orientationSnd]));
   ccsrState.action = NO_ACTION;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));

   // Did sweep and never encountered visual target objects
   return 0;
} 

/*
// Sweep a range of 'angle' degrees in front, capture sonar distance and ambient light. 
// If angle is 360, do a full
circle.
void orientation_static(int angle) {
   int heading_orig;
   int heading_right;
   int heading_left;
   int angle_half;
   char full_circle;
   int x;

   ccsrState.action = ORIENTATION;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
   write(pipeSoundGen[IN], &sound[orientationSnd], sizeof(sound[orientationSnd]));
   
   angle_half = angle/2;
   full_circle = 0;
   
   
   if(angle == 360) {
      // Full left turn circle sweep
      full_circle = 1;
   }
   
   heading_orig = ccsrState.heading;
   heading_right = heading_orig + angle_half;
   heading_left  = heading_orig - angle_half;

if(heading_right>360) {
	heading_right = heading_right - 360;
}
if(heading_left<0) {
	heading_left = heading_left + 360;
}
   printf("hr %d hl %d  \n", heading_right, heading_left);



   // Start right turn to start of scanning angle, no scanning yet
   ccsrState.targetHeading = heading_right;
   turnToTargetHeading(NOSCAN);
   // Start left turn turn, scan
   write(pipeSoundGen[IN], &sound[orientationSnd], sizeof(sound[orientationSnd]));
   ccsrState.targetHeading = heading_left;
   turnToTargetHeading(SCAN);

   write(pipeSoundGen[IN], &sound[orientationSnd], sizeof(sound[orientationSnd]));
   ccsrState.action = NO_ACTION;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));

} 

*/


// Spin down motors to stop, pause until red button is pushed again. If red button is pushed for 3 sec,
// terminate CCSR 
void actionPause() {
   int x;
   int targetSpeedMotor1, targetSpeedMotor2;
   int speed, delta;

   ccsrState.action = PAUSED;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));

   // remember current speeds
   targetSpeedMotor1 = ccsrState.speedMotor1;
   targetSpeedMotor2 = ccsrState.speedMotor2;

   // Slow to stop
   while(!speedFiltered(0, 0)) {
     usleep(BRAIN_LOOP_INTERVAL);
   }
   if(ccsrState.button0Pressed) {
      x=0;
      while(ccsrState.button0Pressed && (x<BUTTON_HOLD_TIME)) {
         x=x+1;
         lcdEvent = EVENT_HOLD_TO_TERMINATE;
	 if(x==20) {
         write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
	 }
	 if(x>BUTTON_HOLD_TIME) {
            x=BUTTON_HOLD_TIME;
         }
         usleep(BRAIN_LOOP_INTERVAL);
      }
      if(x==BUTTON_HOLD_TIME){
 	 ccsrTerminate();
      }
   }
   while(!ccsrState.button0Pressed) {
   }
   while(ccsrState.button0Pressed) {
   }


   // Speed back up to pre-pause state
   speed = (targetSpeedMotor1 + targetSpeedMotor2)/2;
   delta = targetSpeedMotor1 - targetSpeedMotor2;

   while(!speedFiltered(speed, delta)) {
      brainCycle();
   }


   ccsrState.action = NO_ACTION;
   lcdEvent = EVENT_ACTION;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
}



// Detect the minimum turn speed for the terrain we're on. Start right turn with increasing motor power,
// stop when gyro detects movement.
void getMinimumTurnSpeed() {

   int turnSpeed;
   int gyroOn_prev;
   
   turnSpeed = 50;
   gyroOn_prev = ccsrState.gyroOn;
   ccsrState.gyroOn = 1;

   say("Checking terrain conditions");
   while(abs(ccsrState.gyroAngMoment_Y) < 1000) {
      while(!speedFiltered(0, turnSpeed)){
         brainCycle();
      }
      usleep(100000);
      turnSpeed = turnSpeed + 10;
      if (turnSpeed > MAX_MOTOR_TURNSPEED) {
         printf("Can't turn, we're stuck...\n");
         break;
      }
   }

   // Stop turn
   while(!speedFiltered(0, 0)) {
      brainCycle();
   }
   ccsrState.minMotorTurnSpeed = turnSpeed;

   printf("Minimum Turnspeed: %d\n", ccsrState.minMotorTurnSpeed);
   ccsrState.gyroOn = gyroOn_prev;
   
}

// From folded position, extend arm and pick up object from floor, lift it up for the camera to see
// We assume the object is in a prefixed location, and is of a pre-fixed size such that the gripper can grap it
// This will be the result of CCSR manouvering such that the object is in prefixed location, using object tracking
void pickUpObject() {

}

// From folded position, extend arm and open grabber. This is the first step in handing CCSR an object for him to 
// analyze.
void extendArm() {
   setArm(45, 100, 0, 0, 90);  // Fold if not already folded
   setArm(15, 180, 180, 0, 90);  // Extend elbow
}
void dropAndFoldArm() {
   setArm(25, 80, 180, 150, 90);
   setArm(25, 80, 180, 0, 90);
   setArm(45, 80, 90, 0, 90);
   setArm(45, 5, 0, 0, 90);
}


// From folded position, extend arm and open grabber. Move camera to center grabber in visual's 'ROI' box. 
// Wait untill we detect an object being held between grabber fingers (color-based motion detection),
// then close grabber. Bring hand in front of camera. 
// Detect what the color of the held object is. Capture this 
// color as tracked object, and state (if possible) the closest name of the color.
void analyzeObject() {
   char *colorName;
   char objectDetected;
   colorType color;
   
   objectDetected=0;
   say("analyzing object");

   extendArm();           // Extend arm from folded position, and open grabber
   setPanTilt(0,-30,20);  // Move camera to center grabber into image 'region of interest'
   say("please hand me the object");
   
   // Enable object analyzing: this will cause the *visual process in visual.cpp to calculate and
   // store the average color of a region of interest. This captured color will be representing
   // an 'empty, open grabber', or more accurately, whatever is in the background beyond grabber.
   ccsrState.analyzeObject = 1; 
   
   // This a one-shot operation: wait until *visual sets this back to 0, analysis will have been done on one frame
   while (ccsrState.analyzeObject) {
      usleep(100);
   }
   
   // Store this captured 'empty' color, so we can compare it later. We add a window of HSV values to create
   // some hysteresis to reduce mismatches due to moise.
   color.iLowH  = ccsrState.analyzedObjectH - HUE_WINDOW;
   color.iHighH = ccsrState.analyzedObjectH + HUE_WINDOW;
   color.iLowS  = ccsrState.analyzedObjectS - SATURATION_WINDOW;
   color.iHighS = ccsrState.analyzedObjectS + SATURATION_WINDOW;
   color.iLowV  = ccsrState.analyzedObjectV - VALUE_WINDOW;
   color.iHighV = ccsrState.analyzedObjectV + VALUE_WINDOW;

   // Clip
   if (color.iLowH < 0) 	      color.iLowH  = 0;
   if (color.iHighH > MAX_HUE)	      color.iHighH = MAX_HUE;
   if (color.iLowS < 0) 	      color.iLowS  = 0;
   if (color.iHighS > MAX_SATURATION) color.iHighS = MAX_SATURATION;
   if (color.iLowV < 0)               color.iLowV  = 0;
   if (color.iHighV > MAX_VALUE)      color.iHighV = MAX_VALUE;
 
   // Now we wait untill we detect a different color in the roi, which means we will have 'handed' CCSR an
   // object by holding it between its grabber fingers.
   while(!objectDetected){
      // capture new color
      ccsrState.analyzeObject = 1; 
   
      // Wait untill visual has captured color
      while (ccsrState.analyzeObject) {
         usleep(100);
      }

      // Detect and break while loop if new color is different than old. This event means we are offering CCSR
      // something to grab. Note this is inaccurate, we may be changing a color behind the grabber, visual
      // does not see depth yet.
      if((ccsrState.analyzedObjectH<color.iLowH)  ||
     	 (ccsrState.analyzedObjectH>color.iHighH) ||
     	 (ccsrState.analyzedObjectS<color.iLowS)  ||
     	 (ccsrState.analyzedObjectS>color.iHighS) ||
     	 (ccsrState.analyzedObjectV<color.iLowV)  ||
     	 (ccsrState.analyzedObjectV>color.iHighV)){
     	 objectDetected=1;
         say("thank you!");
      }
      sleep(1);  // Wait a little before doing next color capture
   }
   // We detected a change, wait a little and then grab it:
   sleep(1);
   setArm(15, 180, 180, 150, 90);  // close grabber
   // The color captured may be transitionary: we captured a 'blur' when object moved in, or it may
   // be affected by human fingers holding the object. So
   // we wait a little, assuming user will 'steady' the object between grabber, and capture one more time.

   sleep(2);
   ccsrState.analyzeObject = 1;
   // Wait untill visual has captured color
   while (ccsrState.analyzeObject) {
      usleep(100);
   }
   // We have captured object color! 
   write(pipeSoundGen[IN], &sound[evasiveActionSnd], sizeof(sound[evasiveActionSnd]));   //Make sound
 
   // Lookup color name and say it outloud
   colorName = lookupColor(ccsrState.analyzedObjectH,
			   ccsrState.analyzedObjectS,
 			   ccsrState.analyzedObjectV);
   if(colorName!=0){
      say("this object is");
      say(colorName);
   }
   else{
      say("i don't know the color of this object");
   }
   // Set target color range to analysed object plus
   ccsrState.targetColor_iLowH  =  ccsrState.analyzedObjectH - HUE_WINDOW;
   ccsrState.targetColor_iHighH =  ccsrState.analyzedObjectH + HUE_WINDOW;
   ccsrState.targetColor_iLowS  =  ccsrState.analyzedObjectS - SATURATION_WINDOW;
   ccsrState.targetColor_iHighS =  ccsrState.analyzedObjectS + SATURATION_WINDOW;
   ccsrState.targetColor_iLowV  =  ccsrState.analyzedObjectV - VALUE_WINDOW;
   ccsrState.targetColor_iHighV =  ccsrState.analyzedObjectV + VALUE_WINDOW;

   // clip
   if (ccsrState.targetColor_iLowH < 0) ccsrState.targetColor_iLowH		   = 0;
   if (ccsrState.targetColor_iHighH > MAX_HUE) ccsrState.targetColor_iHighH	   = MAX_HUE;
   if (ccsrState.targetColor_iLowS < 0) ccsrState.targetColor_iLowS		   = 0;
   if (ccsrState.targetColor_iHighS > MAX_SATURATION) ccsrState.targetColor_iHighS = MAX_SATURATION;
   if (ccsrState.targetColor_iLowV < 0) ccsrState.targetColor_iLowV                = 0;  
   if (ccsrState.targetColor_iHighV > MAX_VALUE) ccsrState.targetColor_iHighV      = MAX_VALUE;

   // For now we only analyze test-object of knows size: 
   ccsrState.targetColorVolume = TEST_OBJECT_1_VOLUME;

   setPanTilt(0,0,20);  // Move camera to center grabber into image 'region of interest'
   dropAndFoldArm();

}
