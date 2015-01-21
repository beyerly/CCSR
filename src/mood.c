#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include "ccsr.h"
#include "utils.h"
#include "sound.h"
#include "irSensors.h"
#include "mood.h"
#include "facial.h"
#include "servoCtrl.h"



extern FILE *logFile;
extern ccsrStateType ccsrState;
extern int devRandom;
extern int pipeFacialMsg[2];
expressionType expr;


void moodInit() {
   ccsrState.happiness    = 0;
   ccsrState.arousal   = 0;
   ccsrState.randomEyeMovements = 0;
   ccsrState.blinkRate     = 5;
   ccsrState.eyeMovementRate = 10;
   expr.type = EXPR_LOOKSTRAIGHT;
   write(pipeFacialMsg[IN], &expr,sizeof(expr));
}


void *mood() {
   char random;
   int blinkCount;
   int eyeMovementCount;
   expressionType expr;
   int R, G, B;

   logMsg(logFile, "Starting mood maintenance", LOG); 
   blinkCount = ccsrState.blinkRate;
   eyeMovementCount = ccsrState.eyeMovementRate;

   while(1) {

      if(ccsrState.happiness > MIN_HAPPINESS) {
	 ccsrState.happiness  = ccsrState.happiness - 1;
      }
      if(ccsrState.arousal > MIN_AROUSAL) {
	 ccsrState.arousal = ccsrState.arousal - 1;
      }
   
      G = 0;
      B = 0;
      R = ccsrState.arousal;
      if(ccsrState.happiness>0){
	 G = ccsrState.happiness;
      }
      if(ccsrState.happiness<0){
	 B = abs(ccsrState.happiness);
      }
      setRGBLED(R, G, B, 90);
      
      if(ccsrState.randomEyeMovements){
	 if(blinkCount==0){
	    expr.type = EXPR_BLINK;
	    write(pipeFacialMsg[IN], &expr,sizeof(expr));
	    if(read(devRandom, &random, 1) != 1) {
	       logMsg(logFile, "Unsuccessful read from /dev/random", ERROR);	   
	    }
	    blinkCount = (ccsrState.blinkRate * abs(random)/128);
	 }
	 else{
	    blinkCount = blinkCount - 1;
	 }
	 if(eyeMovementCount==0){
	    if(read(devRandom, &random, 1) != 1) {
	       logMsg(logFile, "Unsuccessful read from /dev/random", ERROR);	   
	    }
	    eyeMovementCount = (ccsrState.eyeMovementRate * abs(random)/128);
	    if(read(devRandom, &random, 1) != 1) {
	       logMsg(logFile, "Unsuccessful read from /dev/random", ERROR);	   
	    }
	    expr.type = ((EXPR_LOOKDOWN - EXPR_LOOKSTRAIGHT) * abs(random)/128) + EXPR_LOOKSTRAIGHT;
	    write(pipeFacialMsg[IN], &expr,sizeof(expr));
	 }
	 else{
	    eyeMovementCount = eyeMovementCount - 1;
	 }
      }
      usleep(MOOD_PERIOD);
   }
}

