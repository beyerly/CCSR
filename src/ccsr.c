//############################################################################################
// 
// CCSR Robot project: http://letsmakerobots.com/robot/project/ccsr
//
// Main: Start up all brain functions as pthread processes and run main brain state-machine
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
#include <poll.h>
#include <pthread.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include "ccsr.h"
#include "utils.h"
#include "motor.h"
#include "actions.h"
#include "irSensors.h"
#include "sound.h"
#include "mood.h"
#include "vocal.h"
#include "powerMonitor.h"
#include "navigation.h"
#include "lcdDisp.h"
#include "telemetry.h"
#include "servoCtrl.h"
#include "gpio.h"
#include "visual.h"

FILE *logFile;
int i2cbus, devRandom;
     
ccsrStateType ccsrState = {0};
pthread_mutex_t semI2c;
pthread_mutex_t semAudio;


pthread_t   threadSonar, 
	    threadBlinkLed,
	    threadNavigation,
	    threadPollGpio,
	    threadProximitySensors,
	    threadEnvironmentalSensors,
	    threadSonarSensors,
	    threadSoundGenerator,
	    threadMood,
	    threadVocal,
	    threadVisual,
	    threadCamTrack,
	    threadLcdRefresh,
	    threadLcdmanager,
	    threadEars,
	    threadTelemetry,
	    threadNLP,
	    threadPowerMonitor;


int  pipeSoundGen[2];
int  pipeLCDMsg[2];
char lcdEvent;
soundType sound[standardSoundsCount];


int main () {
   char string[MSG_STRING_SIZE];

   initGpio();
   initSounds();
   initColors();
   initEspeak();
// initCamera();  // mjpg_streamer, obsolete because Im running it separately
   ccsrState.remoteControlled	       = 1;
   ccsrState.minMotorTurnSpeed	       = SLOW_MOTOR_TURNSPEED;
   ccsrState.maxOperatingCurrent       = MAX_OPERATING_CURRENT;
   ccsrState.compassCalibrationOffsetX = 744;
   ccsrState.compassCalibrationOffsetY = -313;
   setTargetColorRange(111, 120, 160, 208, 89, 223);  // Set object tracking color to blue
    
   logFile = fopen(LOG_FILE, "w");
   if (logFile!=NULL) {
      logMsg(logFile, "CCSR start", LOG); 
   }	   
   else {
      printf("CCSR: Could not open %s\n", LOG_FILE);
      exit(EXIT_FAILURE);
   }
   if(pipe(pipeSoundGen) == -1) {
      logMsg(logFile, "Could not open pipeSoundGen", ERROR);
   }
   if(pipe(pipeLCDMsg) == -1) {
      logMsg(logFile, "Could not open pipeLCDMsg", ERROR);
   }
   devRandom = open("/dev/random", O_RDWR);

   pthread_mutex_init(&semI2c, NULL);
   pthread_mutex_init(&semAudio, NULL);
   i2cbus = open(I2C_BUS, O_RDWR);
   if (i2cbus<0) {
    strcpy(string, "Could not open ");
    logMsg(logFile, strcat(string, I2C_BUS), ERROR);
   }

   configServoControl();
   initMotors();

   if(pthread_create( &threadNavigation, NULL, navigation, NULL )) {
      logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadPollGpio, NULL, pollGpio, NULL )) {
      logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadProximitySensors, NULL, proximitySensors, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadEnvironmentalSensors, NULL, environmentalSensors, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
//   if(pthread_create( &threadSonarSensors, NULL, sonarSensors, NULL )) {
//     logMsg(logFile, "Pthread can't be created", ERROR);
//   }
   if(pthread_create( &threadSoundGenerator, NULL, soundGenerator, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadMood, NULL, mood, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadVisual, NULL, visual, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadCamTrack, NULL, camtrack, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadLcdRefresh, NULL, lcdRefresh, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadLcdmanager, NULL, lcdManager, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadEars, NULL, ears, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadTelemetry, NULL, cmdInterface_telemetry, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadNLP, NULL, cmdInterface_NLP, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }
   if(pthread_create( &threadPowerMonitor, NULL, powerMonitor, NULL )) {
     logMsg(logFile, "Pthread can't be created", ERROR);
   }

   write(pipeSoundGen[IN], &sound[linuxBooted], sizeof(sound[linuxBooted]));

   lcdEvent = EVENT_LINUX_BOOTED;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));

//   while(!ccsrState.button0Pressed && ccsrState.remoteControlled) {
//      usleep(10000);
//   }
//   ccsrState.remoteControlled = 0 ;
   lcdEvent = EVENT_CCSR_STARTED;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
   write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[singleA]));
   while(ccsrState.button0Pressed) {
      usleep(10000);
   }
   while(1) {
      switch(ccsrState.state) {
   	 case SM_RESET:
            sleep(1);
            stateChange(SM_REMOTE_CONTROLLED);
            sleep(1);
   	 break;
   	 case SM_DIAGNOSTICS:
   	    if (diagnostics() != 1) {
   	       logMsg(logFile, "Diagnostics failed", ERROR);
   	    };
            stateChange(SM_ORIENTATION);

   	 break;
   	 case SM_ORIENTATION:
   	    orientation(FULL);
	    usleep(2000000);
            if(ccsrState.trackTargetColorOn) {
	       if (ccsrState.objectTracked) {
	          // Wait untill cam is centered on tracked object
		  while(!ccsrState.trackedObjectCentered) {
 	             printf("centering\n");
		     brainCycle();
	          }
                  // Turn to where cam is pointed at
		  ccsrState.targetHeading = addAngleToHeading(ccsrState.pan);
	       }
	       else {
	          // We lost the object from view, continue orientation
	          ccsrState.trackTargetColorOn = 0;
	       }
	    }
//	    else if (ccsrState.fear > 0) {
//	       ccsrState.targetHeading = minLightHeading();
//	    }
	    else {
               // Not tracking visual object, just turn to where we have the most room  
	       ccsrState.targetHeading = deepestSonarDepthHeading();
   	    }
	    turnToTargetHeading(NOSCAN);
            if(ccsrState.trackTargetColorOn &&
	       ccsrState.objectTracked) {
	       // We turned towards tracked object, wait until cam centers again
	       while(!ccsrState.trackedObjectCentered) {
 	          brainCycle();
	       }
	    }
             // Set timer alarm  60 sec from now;
            ccsrState.timerAlarm[0] = ccsrState.timer + 60000000/BRAIN_LOOP_INTERVAL;
 	    if(ccsrState.remoteControlled) {
 	       /* Override normal state progression, force to RC */
 	       stateChange(SM_REMOTE_CONTROLLED);
 	    }
	    else {
//             ccsrState.sonarDistDownFront must be < X to make sure there's no precipice
	       
	       stateChange(SM_DRIVE_TO_TARGET);
	    }
   	 break;
   	 case SM_DRIVE_TO_TARGET:
//	    if(ccsrState.trackTargetColorOn) {
	       if(ccsrState.objectTracked) {
	          ccsrState.trackTargetColorOn = 1;
		  // We are tracking visual object, camera will track object while we drive
		  // Continue driving ahead while cam is pointed forward, otherwise ajust heading 
		  if(abs(ccsrState.pan) < 10) {
		     // Within margin of 10 pix, we consider we are on target
		     speedFiltered(ccsrSpeed(), 0);
 		     if(ccsrStateRest()) {
 		     	// Ran into object and are stopped. Evasive action, will most likely lose track of object
			evasiveAction();
 		     }
		     if(ccsrState.targetVisualObject_Vol > 2000000) {
 		     	// Object is so big, it must be very close, consider target is reached: stop
			while(!speedFiltered(0, 0)) {
 		     	   brainCycle();
 		     	}
 		     	usleep(3000000);
		     	say("Arrived at target object!");
 		     	stateChange(SM_OBSERVE);
		     }
		  }
		  else {
		     // We are tracking object, but cam panned sideways, so we are off course. Stop
		     while(!speedFiltered(0, 0)) {
 		  	brainCycle();
 		     }
		     // Re-aliogn with camera direction
		     ccsrState.targetHeading = addAngleToHeading(ccsrState.pan);
		     turnToTargetHeading(NOSCAN);
		     // Wait until cam senters again
		     while(!ccsrState.trackedObjectCentered) {
 		  	brainCycle();
		     }
		  }
	       }
	       else {
		  // We were tracking, but lost object from sight, go back to orientation to find it
   	          speedFiltered(ccsrSpeed(), 0);
//		   ccsrState.trackTargetColorOn = 0;
//		  while(!speedFiltered(0, 0)) {
 //		     brainCycle();
 //		  }
//	          stateChange(SM_ORIENTATION);
	       }
//	    }  
	    // We are not tracking, look at other conditions
	    if(ccsrState.remoteControlled) {
 	       /* Override normal state progression, force to RC */
   	       speedFiltered(0, 0);
 	       stateChange(SM_REMOTE_CONTROLLED);
 	    }
	    else if(ccsrStateRest()) {
 	       evasiveAction();
 	    }

	    else if(ccsrState.timerAlarm[0] < ccsrState.timer) {
               while (!speedFiltered(0, 0)) {
 	          brainCycle();
 	       }
               stateChange(SM_ORIENTATION);           
	    }
	    else if(ccsrState.stress > 3) {
               while(!speedFiltered(0, 0)) {
 	          brainCycle();
 	       }
   	      orientation(FORWARD_ONLY);
 	      if(ccsrState.trackTargetColorOn) {
	         if (ccsrState.objectTracked) {
	            while(!ccsrState.trackedObjectCentered) {
 	               brainCycle();
	            }
 	            ccsrState.targetHeading = ccsrState.heading + ccsrState.pan;
	         }
	         else {
	            // We lost the object from view, continue orientation
	            ccsrState.trackTargetColorOn = 0;
	         }
              }
	      else {
	         ccsrState.targetHeading = deepestSonarDepthHeading();
	      }
	      turnToTargetHeading(NOSCAN);
 	      if(ccsrState.trackTargetColorOn &&
	         ccsrState.objectTracked) {
	         while(!ccsrState.trackedObjectCentered) {
 	            brainCycle();
	         }
	      }
	      ccsrState.stress = 0;
	    }
	    else if ((ccsrState.ambientLight > 500) &&
	             (ccsrState.fear == 0)) {
 	       while(!speedFiltered(0, 0)) {
 	          brainCycle();
 	       }
	       ccsrState.targetHeading = addAngleToHeading(-90);
     	       say("Found sufficient ambient light!");
	       turnToTargetHeading(NOSCAN);
               usleep(3000000);
	       stateChange(SM_OBSERVE);
            }	    
/*	    else if ((ccsrState.ambientLight < 50) &&
	             (ccsrState.fear > 0)) {
 	       while(!speedFiltered(0, 0)) {
 	          brainCycle();
 	       }
	       ccsrState.targetHeading = ccsrState.heading - 90;
	       if(ccsrState.targetHeading<0) {
	          ccsrState.targetHeading = ccsrState.targetHeading+360;
	       }
     	       turnToTargetHeading(NOSCAN);
               stateChange(SM_OBSERVE);
            }
*/	    
            else {
   	       speedFiltered(ccsrSpeed(), 0);
	    }
   	 break;
   	 case SM_OBSERVE:
 	    if(ccsrState.remoteControlled) {
 	       /* Override normal state progression, force to RC */
 	       stateChange(SM_REMOTE_CONTROLLED);
 	    }
	    else if(ccsrState.motionDetected ||
	       ccsrState.noiseDetected) {
               ccsrState.fear = ccsrState.fear + 60; // Add 10 sec of fear   
               stateChange(SM_ORIENTATION);
	    }
//	    else if ((ccsrState.ambientLight < 50) &&
//	             (ccsrState.fear == 0)) {
//               stateChange(SM_ORIENTATION);
//	    }	     
   	 break;
   	 case SM_REMOTE_CONTROLLED:
 	    if(!ccsrState.remoteControlled) {
 	       stateChange(SM_ORIENTATION);
 	    }
   	 break;
   	 case SM_EXPLORE:
	    // Basic obstacle avoidance
	    if(speedFiltered(ccsrSpeed(), 0) && (ccsrSpeed()==0)) {
 	       usleep(1000000);
	       evasiveAction();
 	    }
	    if(ccsrState.remoteControlled) {
 	       /* Override normal state progression, force to RC */
   	       while(!speedFiltered(0, 0)) {
                  brainCycle();
	       }
 	       stateChange(SM_REMOTE_CONTROLLED);
 	    }
   	 break;
      }
      brainCycle();
//    printf("t %d %d %d %d %d\n", ccsrState.timer, ccsrState.timerAlarm[0], ccsrState.motionDetected, ccsrState.noiseDetected, ccsrState.state_next);
   }
}



void stateChange(char state) {
   ccsrState.state = state;

   switch(state) {
      case SM_OBSERVE:
	 ccsrState.proximitySensorsOn	  = 0;
	 ccsrState.sonarSensorsOn	  = 0;
	 ccsrState.environmantalSensorsOn = 1;
	 ccsrState.navigationOn 	  = 0;
	 ccsrState.pidMotionDetectOn	  = 1;
	 ccsrState.noiseDetectOn	  = 1;
	 write(pipeSoundGen[IN], &sound[observeSnd], sizeof(sound[observeSnd]));
      break;
      case SM_DRIVE_TO_TARGET:
	 ccsrState.proximitySensorsOn	  = 1;
	 ccsrState.sonarSensorsOn	  = 0;
	 ccsrState.environmantalSensorsOn = 1;
	 ccsrState.navigationOn 	  = 1;
	 ccsrState.pidMotionDetectOn	  = 0;
	 ccsrState.noiseDetectOn	  = 0;
      break;
      case SM_ORIENTATION:
	 ccsrState.proximitySensorsOn	  = 0;
	 ccsrState.sonarSensorsOn	  = 1;
	 ccsrState.environmantalSensorsOn = 0;
	 ccsrState.navigationOn 	  = 1;
	 ccsrState.pidMotionDetectOn	  = 0;
	 ccsrState.noiseDetectOn	  = 0;
	 ccsrState.trackTargetColorOn     = 0;
      break;
       case SM_EXPLORE:
	 ccsrState.proximitySensorsOn	  = 1;
      break;
  }
   lcdEvent = EVENT_SM_STATE_CHANGE;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
}



void ccsrTerminate() {
/*      pthread_join(threadCheckButtons, NULL);
      pthread_join(threadSonar, NULL); 
      pthread_join(threadNavigation, NULL);
      pthread_join(threadCheckButtons, NULL);
      pthread_join(threadProximitySensors, NULL);
      pthread_join(threadEnvironmentalSensors, NULL);
      pthread_join(threadSonarSensors, NULL);
      pthread_join(threadSoundGenerator, NULL);
      pthread_join(threadMood, NULL);
      pthread_join(threadVisual, NULL);
//      pthread_join(threadVocal, NULL);
      pthread_join(threadLcdRefresh, NULL);
      pthread_join(threadLcdmanager, NULL);
      pthread_join(threadEars, NULL);
      pthread_join(threadPowerMonitor, NULL);
*/
 
      write(pipeSoundGen[IN], &sound[terminate], sizeof(sound[terminate]));
      lcdEvent = EVENT_TERMINATE;
      write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
      // Set servo's in neutral position
      setPanTilt(0, 0, 50);
      printf("Time run: %d brain cycles\n", ccsrState.timer);
      usleep(5000000);
      exit(0); 
}



