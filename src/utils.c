

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <math.h>
#include <poll.h>
#include "ccsr.h"
#include "irSensors.h"
#include "motor.h"
#include "lcdDisp.h"
#include "sound.h"
#include "actions.h"
#include "powerMonitor.h"


extern FILE *logFile;
extern ccsrStateType ccsrState;
extern int pipeLCDMsg[2];
char lcdEvent;
extern soundType sound[NUM_SOUNDS];
extern int pipeSoundGen[2];

int logMsg(FILE* logFile, char* s, int type) {
   switch (type) {
      case LOG:   {
         fprintf(logFile, "CCSR LOG: %s \n", s); 
	 break;
      }
      case ERROR: {
         fprintf(logFile, "CCSR ERROR: %s \n", s); 
	 break;
      }
   }
}

int logState(FILE* logFile, ccsrStateType state) {
   fprintf(logFile, "CCSR State: \n"); 
}

void brainCycle() {
   /* This function must get called every BRAIN_LOOP_INTERVAL u-secs. Any signal trapping should be done here */
   ccsrState.timer = ccsrState.timer + 1; 
   if(ccsrState.button0Pressed) {
      actionPause();
   }
   usleep(BRAIN_LOOP_INTERVAL);
}


/*
void *blinkLed() {
   char string[MSG_STRING_SIZE];


   logMsg(logFile, "Starting LED blink", LOG); 
#ifdef CCSR_PLATFORM
   minnowLed1t = fopen(MINNOW_LED1t, "w");
   if (minnowLed1t!=NULL) {
      fputs ("none", minnowLed1t);
   }	   
   else {
      strcpy(string, "Could not open ");
      logMsg(logFile, strcat(string, MINNOW_LED1t), ERROR); 
   }
   while(1) { 
      minnowLed1b = fopen(MINNOW_LED1b, "w");
      if (minnowLed1b!=NULL) {
	 fputs ("1", minnowLed1b);
	 fflush(minnowLed1b);
 	 usleep(500000);
	 fputs ("0", minnowLed1b);
	 fflush(minnowLed1b);
	 usleep(500000);
      }
      else {
         strcpy(string, "Could not open ");
         logMsg(logFile, strcat(string, MINNOW_LED1b), ERROR); 
      }
   }   
   fclose(minnowLed1b);
   fclose(minnowLed1t);
#endif
}
*/

int diagnostics() {
   int fail = 0;
   int OnPrev;
   int time, timeout;
   time = 0;
   timeout = 5; // 5 sec
   
   logMsg(logFile, "Starting Diagnostics", LOG); 
   
   fail |=  !motorDiagnostic();
   fail |=  !pantiltDiagnostics();
   
   // Sound diagnostics:
   OnPrev = ccsrState.noiseDetectOn;
   // Wait untill CCSR is done talking
//   drainAudioPlayback();
   ccsrState.noiseDetectOn = 1;
   usleep(1000000);
   while(!ccsrState.noiseDetected && (time < timeout)) {
      write(pipeSoundGen[IN], &sound[doubleA], sizeof(sound[doubleA]));
      usleep(1000000);
      time = time + 1;
   }
   if(time >= timeout) {
      printf("Diagnostic error, sound test timed out\n");   
   }  
   ccsrState.noiseDetectOn = OnPrev;

   // Motion Detection diagnostics:
   OnPrev = ccsrState.pidMotionDetectOn;
   ccsrState.pidMotionDetectOn = 1;
   time = 0;
   say("Testing motion detection, please move!"); 
   usleep(1000000);
   while(!ccsrState.motionDetected && (time < timeout)) {
      usleep(1000000);
      time = time + 1;
   }
   if(time >= timeout) {
      printf("Diagnostic error, motion test timed out\n");   
   }  
   ccsrState.pidMotionDetectOn = OnPrev;
   say("Diagnostics successfull!"); 
  
   return !fail;
}

// return 1 if CCSR is sitting at rest for a pre-defined amount of time. 
int ccsrStateRest() {

   if((ccsrState.speedMotor1 == 0) && (ccsrState.speedMotor2 == 0)) {
      ccsrState.timeAtRest++;
      if(ccsrState.timeAtRest>EVASIVE_ACTION_DELAY) {
	 ccsrState.timeAtRest = EVASIVE_ACTION_DELAY;
      }
   }
   else {
      ccsrState.timeAtRest=0;
   }
   if(ccsrState.timeAtRest >= EVASIVE_ACTION_DELAY) {
//      ccsrState.timeAtRest=0;
      return 1;
   }
   else {
      return 0;
   }   
}


int min(int a, int b) {
     	if(a<b) {
      return a;
   }
   else {
      return b;
   }
}

int max(int a, int b) {
     	if(a>b) {
      return a;
   }
   else {
      return b;
   }
}

// Return forward/backward speed that Squirl can drive at, based on proximity sensors
int ccsrSpeed() {

   int x,z;
   double m, n;
   
   // Normally, base speed on linear function of 2 front ir distance sensors. The more space in front, the slower we go. Stop point
   // is ~15cm before front. Drive backward if distance becomes less than that. 
   
   n = (double) max(ccsrState.irDistFrontLeft, ccsrState.irDistFrontRight); // Note: higher value is closer
   //n = (double) ccsrState.irDistFrontRight;
   // Make linear irSensor output: -255 to 255 
   m = (-(log(n) - log(MAX_IR_RANGE))/log(IR_SENSOR_EXPONENT))  - MOTOR_SPEED_CURVE_OFFSET;
   x = (int) floor(m);   

   // Scale to MAx motor speed
   x = x*MAX_MOTOR_SPEED/255;

   // Clip in case of spikes
   if (x<-MAX_MOTOR_SPEED) {
      x = MAX_MOTOR_SPEED;
   }
   else if( x > MAX_MOTOR_SPEED) {
      x = MAX_MOTOR_SPEED;
   }
   // Quantize to range of []
   x = (x / (MAX_MOTOR_SPEED/QUANT_STEP)) * (MAX_MOTOR_SPEED/QUANT_STEP) ;

   // Below motor threshhold speed, clip to 0 (off)
   if (x<MOTOR_POS_THRESHOLD && x>MOTOR_NEG_THRESHOLD) {
      x = 0;
   }
   
   // If lower sensor shows too much space, we're at a precipice, stop to 0.
   if(ccsrState.irDistBelow < IR_DIST_BOTTOM_THRESHOLD) {
//      x = 0;
   }
   // If sonar finds very close object (missed by ir Sensors), stop to 0.
   if(ccsrState.sonarDistFront < SONAR_DIST_FRONT_THRESHOLD) {
 //     x = 0;
   }
   // If current exceeds limit (for prolonged time), we may be stuck, stop to 0.
   if(ccsrState.currentLimit) {
      x = 0;
   }
   
//   printf("ccsrSpeed %d %d %d %d %d\n", x, ccsrState.irDistFrontLeft, ccsrState.irDistFrontRight, ccsrState.irDistBelow, ccsrState.sonarDistFront, x);
   
   return x;
}


// return turn-speed based on current heading and target heading
int ccsrSpeedDelta(int targetHeading) {
   int delta;
   int motorSpeedDelta;
   
   delta = abs(targetHeading - ccsrState.heading);
   if(delta > 180) {
      delta = 360 - delta; 
   }
   
   if(delta < SLOW_TURNSPEED_THRESHOLD) {
      // Slow down turn is target heading is almost reached
      motorSpeedDelta = ccsrState.minMotorTurnSpeed;
   }
   else {
      motorSpeedDelta = MAX_MOTOR_TURNSPEED;
   }
   
   if(ccsrState.currentLimit) {
      motorSpeedDelta=0;
   }
   
   return motorSpeedDelta;
}

// Return direction (LEFT, RIGHT) to turn for the shortest turn to targetHeading
int shortestTurnDir(int targetHeading) {

   int turnDir;
   int delta;
   
   delta = abs(ccsrState.heading - targetHeading);
   if(delta > 180) {
      delta = 360 - delta;
      if(ccsrState.heading > targetHeading) {
         turnDir = RIGHT;
      }
      else {
         turnDir = LEFT;
      }      
   }
   else {
      if(ccsrState.heading > targetHeading) {
         turnDir = LEFT;
      }
      else {
         turnDir = RIGHT;
      }      
   }
   return turnDir;
}



// Return delta between target heading and current heading.
// Positive if turnDir direction is the shortest path to the target direction, 
// negative otherwise.
int headingDelta(int targetHeading, int turnDir) {

   int shortTurnDir;
   int delta;
   
   delta = abs(ccsrState.heading - targetHeading);
   if(delta > 180) {
      delta = 360 - delta;
   }
      shortTurnDir = shortestTurnDir(targetHeading);

   if (turnDir != shortTurnDir) {
      delta = -delta;
   }
   return delta;
}


int deepestSonarDepthHeading() {
   int x, y;
   int heading;
   int average;
   int samples;
   int maxWidth, width;
   char prevLargerAvg;
   int start;
   int rangeStart;
   
   samples = 0;
   width = 0;
   maxWidth = 0;
   prevLargerAvg = 0;
   start = 0;
   rangeStart = 0;
   average=0;

   // Find average
   for (x=0;x<=360;x++) {
      if(ccsrState.profileValid[x]) {
         average = average +  ccsrState.sonarDistProfile[x];  
         samples = samples + 1;
printf("%d %d\n", x, ccsrState.sonarDistProfile[x]);
      }
   }
   average = average / samples;

   // Find middle of largest area above average
   // Start scan outside of valid area
   x = 0;
   while((ccsrState.sonarDistProfile[x] >= average) &&
          ccsrState.profileValid[x]) {
      x=x+1;
      // This should never hang; at least one sample must be below average.
   }
   y = x - 1;
   if(y<0) {
      y = 360;
   }
   
   while(x!=y) {
      if((ccsrState.sonarDistProfile[x] >= average) &&
          ccsrState.profileValid[x]) {
         if(prevLargerAvg) {
	    width = width + 1;
	 }
	 else {
	    start = x;
	    width = 0;
	 }
         prevLargerAvg = 1;
      }
      else {
         if(prevLargerAvg) {
 	    if(width>maxWidth) {
	       maxWidth = width;
	       rangeStart = start;
	    }
	    
	 }
         prevLargerAvg = 0;
      }
      x = x+1;
      if(x>360) {
         x = 0;
      }
   }
   if(width>maxWidth) {
      maxWidth = width;
      rangeStart = start;
   }
   
      heading = rangeStart + maxWidth/2;
      if(heading>360) {
         heading = heading - 360;
      }
   printf("sonarDepth: avg: %d samples: %d range start %d range width %d heading %d\n", average, samples, rangeStart, maxWidth,
   heading);
      return heading;
}


/*
int deepestSonarDepthHeading() {
   int x;
   int heading;
   int average;
   int samples;
   int validSamples[360];
   int maxWidth, width;
   char prevLargerAvg;
   int start;
   int rangeStart, rangeEnd;
   
   samples = 0;
   width = 0;
   maxWidth = 0;
   prevLargerAvg = 0;
   start = 0;
   rangeStart = 0;
   rangeEnd = 0;
   average=0;
   for (x=0;x<=359;x++) {
      if(ccsrState.profileValid[x]) {
printf(" valid %d\n", x);
         average = average +  ccsrState.sonarDistProfile[x];  
         validSamples[samples] = x;
         samples = samples + 1;
      }
   }
   average = average / samples;
   for (x=0;x<samples;x++) {
      if(ccsrState.sonarDistProfile[validSamples[x]] >= average) {
         if(prevLargerAvg) {
	    width = width + 1;
	 }
	 else {
	    start = validSamples[x];
	    width = 0;
	 }
         prevLargerAvg = 1;
      }
      else {
         if(prevLargerAvg) {
 	    if(width>maxWidth) {
	       maxWidth = width;
	       rangeStart = start;
	       rangeEnd = validSamples[x];
	    }
	    
	 }
         prevLargerAvg = 0;
      }
   }
   if(width>maxWidth) {
      maxWidth = width;
      rangeStart = start;
      rangeEnd = validSamples[samples-1];
   }
   printf("sonarDepth: avg: %d samples: %d range start %d range end %d\n", average, samples, rangeStart, rangeEnd);
   return rangeStart + (rangeEnd - rangeStart)/2;
}
*/

int minLightHeading() {
   int x;
   int heading;
   int average;
   int samples;
   int validSamples[360];
   int maxWidth, width;
   char prevSmallerAvg;
   int start;
   int rangeStart, rangeEnd;
   int target;
      
   samples = 0;
   width = 0;
   maxWidth = 0;
   prevSmallerAvg = 0;
   start = 0;
   rangeStart = 0;
   rangeEnd = 0;
   average=0;
   for (x=0;x<=359;x++) {
      if(ccsrState.profileValid[x]) {
         average = average +  ccsrState.ambientLightProfile[x];  
         validSamples[samples] = x;
         samples = samples + 1;
      }
   }
   average = average / samples;
   for (x=0;x<samples;x++) {
      if(ccsrState.ambientLightProfile[validSamples[x]] <= average) {
         if(prevSmallerAvg) {
	    width = width + 1;
	 }
	 else {
	    start = validSamples[x];
	    width = 0;
	 }
         prevSmallerAvg = 1;
      }
      else {
         if(prevSmallerAvg) {
 	    if(width>maxWidth) {
	       maxWidth = width;
	       rangeStart = start;
	       rangeEnd = validSamples[x];
	    }
	    
	 }
         prevSmallerAvg = 0;
      }
   }
   if(width>maxWidth) {
      maxWidth = width;
      rangeStart = start;
      rangeEnd = validSamples[samples-1];
   }
   printf("minAmbientLight: avg: %d samples: %d range start %d range end %d\n", average, samples, rangeStart, rangeEnd);
   target = rangeStart + (rangeEnd - rangeStart)/2;
   target = target - 90;
   if(target<0) {
      target = target + 360;
   }
   return target;
}



int maxLightHeading() {
   int x;
   int maxLight;
   int heading;
   
   maxLight = 0;
   for (x=0;x<=360;x++) {
      if(ccsrState.profileValid[x] && (ccsrState.ambientLightProfile[x] > maxLight)) {
         heading = x;
         maxLight = ccsrState.ambientLightProfile[x];
      }
   }
// printf("light heading %d is %d \n", heading, maxLight);   
   return heading; 
}




// return sum of 2 angles
int addAngle(int angle, int hdng) {
   int heading;
   
   heading = hdng + angle;
   if(heading>360) {
      heading = heading - 360;
   }
   if(heading<0) {
      heading = heading + 360;
   }
   return heading;
}


// return sum of angle and current heading
int addAngleToHeading(int angle) {
   int heading;
   
   heading = ccsrState.heading + angle;
   if(heading>360) {
      heading = heading - 360;
   }
   if(heading<0) {
      heading = heading + 360;
   }
   return heading;
}

// Return the pan-heading [-90, 90] towards targetHeading. 
// So if current heading is 90, and targetHeading is 100, the pan to turn head towards target heading is 10 degrees
// This clips at max/min pan
int getPanToHeading(int targetHeading){
   int delta;
   delta = targetHeading - ccsrState.heading;
   if(delta > 180) {
      delta = delta - 360;
   }
   else if(delta < -180) {
      delta = 360 + delta;
   }
   if(delta>90){
      delta=90;
   }
   else if(delta<-90){
      delta=-90;
   }
   return delta;
}

// Should never be called: we should read beacons from map.svg!
void initBeacons(){

   // Placeholder beacons: beacon_0 = (-6, -1), beacon_1 = (4, 4), 
   ccsrState.beaconListX[0] = -6;
   ccsrState.beaconListY[0] = -1;
   ccsrState.beaconListName[0] =  (char*) malloc(10*sizeof(char));
   strcpy(ccsrState.beaconListName[0], "BCNblue");

   ccsrState.beaconListX[1] = 4;
   ccsrState.beaconListY[1] = 4;
   ccsrState.beaconListName[1] =  (char*) malloc(10*sizeof(char));
   strcpy(ccsrState.beaconListName[0], "BCNgreen");
}


// Triangulate using 2 beacon observations. Derive linear equations representing 2 lines of sight, then
// find position in intersection of 2 lines
// beaconA/B are indices into a beacon array. headingA and B are heading observations in degrees
// The function will update X and Y with current location
void triangulate(char beaconA, char beaconB, int headingA, int headingB, int* X, int* Y){

   float pi;
   float hA,hB;
   float a,b,c,d;


   // Placeholder beacons: beacon_0 = (-6, -1), beacon_1 = (4, 4), 
   ccsrState.beaconListX[0] = -6;
   ccsrState.beaconListY[0] = -1;
   ccsrState.beaconListX[1] = 4;
   ccsrState.beaconListY[1] = 4;

   pi = 3.14159265;

   // Create:
   //    YA=aX+b
   //    YB=bX+c
   // To find X,Y, solve YA=YB 

   // Bring heading (degrees) to 1st or 2nd quadrant
   if(headingA>180){
      hA=(float) headingA-180;
   }
   hA = 90 - hA;       // Calculate angle with respect to x-axis
   hA = pi*(hA/180);   // Convert to radians
   a = tan(hA);        // Calculate X/Y ratio of angle
   b = ccsrState.beaconListY[beaconA] - a*ccsrState.beaconListX[beaconA]; // Calculate required Y-shift for linear equation to intercept the beacon

   // Bring heading (degrees) to 1st or 2nd quadrant
   if(headingB>180){
      hB=(float) headingB-180;
   }
   hB = 90 - hB;       // Calculate angle with respect to x-axis
   hB = pi*(hB/180);   // Convert to radians
   c = tan(hB);        // Calculate X/Y ratio of angle
   d = ccsrState.beaconListY[beaconB] - a*ccsrState.beaconListX[beaconB]; // Calculate required Y-shift for linear equation to intercept the beacon

   // Solve equation YA=YB
   *X=(int) (d-b)/(a-c);
   *Y=(int) a**X+B;

   // Check if we are on the map?
}

