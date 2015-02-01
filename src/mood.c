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
#include "mood.h"



extern FILE *logFile;
extern ccsrStateType ccsrState;
extern int devRandom;
extern int pipeFacialMsg[2];
expressionType expr;
int moodDegradationCounter;

char moodLUT[][] = {
   { 
      EXPR_ANGRY,
      MOOD_NORMAL,
      MOOD_NORMAL,
      EXPR_HAPPY
   },
   { 
      EXPR_SAD,
      MOOD_NORMAL,
      MOOD_NORMAL,
      MOOD_NORMAL
   },
   { 
      EXPR_SAD,
      MOOD_NORMAL,
      MOOD_NORMAL,
      MOOD_NORMAL
   },
   { 
      EXPR_SLEEP,
      EXPR_SLEEP,
      MOOD_NORMAL,
      MOOD_NORMAL
   }
}

void moodInit() {
   ccsrState.happiness    = 0;
   ccsrState.arousal   = 0;
   ccsrState.randomEyeMovements = 1;
   ccsrState showEmotion = 1;
   ccsrState.blinkRate     = 5;
   ccsrState.eyeMovementRate = 10;
   moodDegradationCounter = MOOD_DEGRADATION_SPEED;
   expr.type = EXPR_LOOKSTRAIGHT;
   write(pipeFacialMsg[IN], &expr,sizeof(expr));
}

void setMood(int h, int a){
   ccsrState.happiness = ccsrState.happiness + h;
   ccsrState.arousal = ccsrState.arousal + a;
   if(ccsrState.happiness > MAX_HAPPINESS){
      ccsrState.happiness = MAX_HAPPINESS;
   }
   if(ccsrState.arousal > MAX_AROUSAL){
      ccsrState.arousal = MAX_AROUSAL;
   }
}

void *mood() {
   char random;
   int blinkCount;
   int eyeMovementCount;
   expressionType expr;
   int R, G, B;
   char xLUT, yLUT;

   logMsg(logFile, "Starting mood maintenance", LOG); 
   blinkCount = ccsrState.blinkRate;
   eyeMovementCount = ccsrState.eyeMovementRate;

   while(1) {
      if(moodDegradationCounter==0){
         if(ccsrState.happiness > MIN_HAPPINESS) {
            ccsrState.happiness  = ccsrState.happiness - 1;
         }
         if(ccsrState.arousal > MIN_AROUSAL) {
            ccsrState.arousal = ccsrState.arousal - 1;
         }
         moodDegradationCounter = MOOD_DEGRADATION_SPEED;
      }
      else{
         moodDegradationCounter = moodDegradationCounter - 1;
      }

      R = ccsrState.arousal;
      G = (ccsrState.happiness + MAX_HAPPINESS)/2;
      B = MAX_HAPPINESS - G;

      if(ccsrState.showEmotion){
         setRGBLED(R, G, B, 90);
      }

      xLUT = 2*(ccsrState.happiness + MAX_HAPPINESS)/MAX_HAPPINESS;
      yLUT = 4*ccsrState.arousal/MAX_AROUSAL;
      printf("mood hap %d ar %d X %d Y %d\n", ccsrState.happiness, ccsrState.arousal, xLUT, yLUT);
      if(moodLUT[yLUT][xLUT] != MOOD_NORMAL){
         if(ccsrState.showEmotion){
            if(moodLUT[yLUT][xLUT] == EXPR_SLEEP){
               if(ccsrState.state != SM_SLEEP) {
                  goToSleep();
               }
            }
            else {
               if(ccsrState.state == SM_SLEEP) {
                  wakeFromSleep();
               }
               expr.type = moodLUT[yLUT][xLUT];
               write(pipeFacialMsg[IN], &expr,sizeof(expr));
            }
         }
      }
      else {
         if(ccsrState.state == SM_SLEEP) {
            wakeFromSleep();
         }
         if(ccsrState.randomEyeMovements){
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
      }
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
      }
      usleep(MOOD_PERIOD);
   }
}

