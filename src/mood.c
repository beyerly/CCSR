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


extern FILE *logFile;
extern ccsrStateType ccsrState;


void *mood() {

   int personalSpace;
   logMsg(logFile, "Starting mood maintenance", LOG); 

   while(1) {
      if(ccsrState.fear > 0) {
         ccsrState.fear = ccsrState.fear - 1;
      }
//      personalSpace = ccsrState.irDistFrontLeft;
//      ccsrState.stress = (10*personalSpace)/MAX_IR_RANGE;

      usleep(MOOD_PERIOD);
   }
}

