#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include "ccsr.h"
#include "utils.h"
#include "sound.h"
#include "powerMonitor.h"
#include "vocal.h"


extern FILE *logFile;
extern ccsrStateType ccsrState;
extern soundType sound[NUM_SOUNDS];
extern int pipeSoundGen[2];

ccsrStateType ccsrStatePrev;

void *vocal() {

   logMsg(logFile, "Starting vocals", LOG); 

   ccsrStatePrev = ccsrState;

   while(1) {
      // Stress level
//      if(ccsrState.stress != ccsrStatePrev.stress) {
//	 if(write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[singleA])) < 1) {
//            logMsg(logFile, "Can't write sound", ERROR); 
//         }
//      }

      // Signal if stopped
//      if(((ccsrState.speedMotor1 != ccsrStatePrev.speedMotor1) || 
//          (ccsrState.speedMotor2 != ccsrStatePrev.speedMotor2)) &&
//	  ccsrStateRest()) {
//	 if(write(pipeSoundGen[IN], &sound[melody_Farpi], sizeof(sound[melody_Farpi])) < 1) {
//            logMsg(logFile, "Can't write sound", ERROR); 
//         }
//      }
//      // Signal if starting to move
//      if(((ccsrState.speedMotor1 != ccsrStatePrev.speedMotor1) || 
//          (ccsrState.speedMotor2 != ccsrStatePrev.speedMotor2)) &&
//	  (ccsrStatePrev.speedMotor1==0) &&
//	  (ccsrStatePrev.speedMotor2==0)) {
//	 if(write(pipeSoundGen[IN], &sound[melody_Farp], sizeof(sound[melody_Farp])) < 1) {
//            logMsg(logFile, "Can't write sound", ERROR); 
//         }
//      }
      // Signal max number of current limit events happened: too much continuous current
      if(ccsrState.currentLimit >= MAX_CURRENTLIMIT_EVENTS) {
	 if(write(pipeSoundGen[IN], &sound[currentAlarm], sizeof(sound[currentAlarm])) < 1) {
            logMsg(logFile, "Can't write sound", ERROR); 
         }
      }
      // Signal max number of low battery events happened: battery needs charging
      if(ccsrState.lowBattery >= MAX_LOW_BATTERY_EVENTS) {
	      if(write(pipeSoundGen[IN], &sound[batteryAlarm], sizeof(sound[batteryAlarm])) < 1) {
            logMsg(logFile, "Can't write sound", ERROR); 
	      }
      }
      
      
      
      ccsrStatePrev = ccsrState;
      usleep(VOCAL_PERIOD);
   }
}

