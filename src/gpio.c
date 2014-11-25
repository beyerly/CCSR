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


FILE *minnowLed1t, 
     *minnowLed1b,
     *gpio244Edg,
     *gpio244Val,
     *gpio244Dir,
     *gpio245Edg,
     *gpio245Val,
     *gpio245Dir,
     *gpio246Edg,
     *gpio246Val,
     *gpio246Dir,
     *gpio247Edg,
     *gpio247Val,
     *gpio247Dir,
     *gpio248Edg,
     *gpio248Val,
     *gpio248Dir,
     *gpio249Edg,
     *gpio249Val,
     *gpio249Dir,
     *gpio250Edg,
     *gpio250Val,
     *gpio250Dir,
     *gpio251Edg,
     *gpio251Val,
     *gpio251Dir;

struct pollfd fds[NUM_BOTTONS];



// GPIO's:

// GPIO244 = gpio7 = 	     Big red button
// GPIO246 = gpio6	     keypad #1: toggle display mode: SHOW_ACTION, SHOW_STATUS, SHOW_EVENT, SHOW MENUE
// GPIO245 = gpio5	     keypad #2: toggle status field:
// GPIO248 = gpio4	     keypad #3: toggle display menue items
// GPIO247 = gpio3	     Busted GPIO!! Only polls on assertion, doesn't detect release
// GPIO249 =  gpio2          Motion detection PID
// GPIO250 =  gpio1          Mic power
// GPIO251 =  gpio0	     keypad #4: toggle current menue value


void initGpio() {
   gpio244Edg = fopen(GPIO244_EDG, "w");
   gpio245Edg = fopen(GPIO245_EDG, "w");
   gpio246Edg = fopen(GPIO246_EDG, "w");
   gpio247Edg = fopen(GPIO247_EDG, "w");
   gpio248Edg = fopen(GPIO248_EDG, "w");
   gpio249Edg = fopen(GPIO249_EDG, "w");
   gpio251Edg = fopen(GPIO251_EDG, "w");
   fputs("both", gpio244Edg); fflush(gpio244Edg);
   fputs("both", gpio245Edg); fflush(gpio245Edg);
   fputs("both", gpio246Edg); fflush(gpio246Edg);
   fputs("both", gpio247Edg); fflush(gpio247Edg);
   fputs("both", gpio248Edg); fflush(gpio248Edg);
   fputs("both", gpio249Edg); fflush(gpio249Edg);
   fputs("both", gpio251Edg); fflush(gpio251Edg);
   gpio244Dir = fopen(GPIO244_DIR, "w");
   gpio245Dir = fopen(GPIO245_DIR, "w");
   gpio246Dir = fopen(GPIO246_DIR, "w");
   gpio247Dir = fopen(GPIO247_DIR, "w");
   gpio248Dir = fopen(GPIO248_DIR, "w");
   gpio249Dir = fopen(GPIO249_DIR, "w");
   gpio251Dir = fopen(GPIO251_DIR, "w");
   fputs ("in", gpio244Dir); fflush(gpio244Dir);
   fputs ("in", gpio245Dir); fflush(gpio245Dir);
   fputs ("in", gpio246Dir); fflush(gpio246Dir);
   fputs ("in", gpio247Dir); fflush(gpio247Dir);
   fputs ("in", gpio248Dir); fflush(gpio248Dir);
   fputs ("in", gpio249Dir); fflush(gpio249Dir);
   fputs ("in", gpio251Dir); fflush(gpio251Dir);

   // Mic amp power
   gpio250Dir = fopen(GPIO250_DIR, "w");
   fputs ("out", gpio250Dir); fflush(gpio250Dir);
   gpio250Val = fopen(GPIO250_VAL, "w");
}



void *pollGpio() {
   char string[MSG_STRING_SIZE];
   int value;
   int c;
   logMsg(logFile, "Starting checkButtons", LOG); 
#ifdef CCSR_PLATFORM

int i;

   while(1) {
      gpio244Val = fopen(GPIO244_VAL, "r+");
      gpio245Val = fopen(GPIO245_VAL, "r+");
      gpio246Val = fopen(GPIO246_VAL, "r+");
      gpio247Val = fopen(GPIO247_VAL, "r+");
      gpio248Val = fopen(GPIO248_VAL, "r+");
      gpio249Val = fopen(GPIO249_VAL, "r+");
      gpio251Val = fopen(GPIO251_VAL, "r+");
      fds[0].fd = fileno(gpio244Val);
      fds[0].events =  POLLPRI;
      fds[1].fd = fileno(gpio245Val);
      fds[1].events =  POLLPRI;
      fds[2].fd = fileno(gpio246Val);
      fds[2].events =  POLLPRI;
      fds[3].fd = fileno(gpio247Val);
      fds[3].events =  POLLPRI;
      fds[4].fd = fileno(gpio248Val);
      fds[4].events =  POLLPRI;
      fds[5].fd = fileno(gpio249Val);
      fds[5].events =  POLLPRI;
      fds[6].fd = fileno(gpio251Val);
      fds[6].events =  POLLPRI;
      fgetc(gpio244Val);
      fgetc(gpio245Val);
      fgetc(gpio246Val);
      fgetc(gpio247Val);
      fgetc(gpio248Val);
      fgetc(gpio249Val);
      fgetc(gpio251Val);
      if(poll(fds, 7, -1) > 0) {
         fseek(gpio244Val, 0, SEEK_SET);
         fseek(gpio245Val, 0, SEEK_SET);
         fseek(gpio246Val, 0, SEEK_SET);
         fseek(gpio247Val, 0, SEEK_SET);
         fseek(gpio248Val, 0, SEEK_SET);
         fseek(gpio249Val, 0, SEEK_SET);
         fseek(gpio251Val, 0, SEEK_SET);
         if(fgetc(gpio244Val) ==  (int) '1') {
            // Big red button
            if(ccsrState.button0Pressed == 0) {
               write(pipeSoundGen[IN], &sound[keyPressed], sizeof(sound[keyPressed]));
            }
            ccsrState.button0Pressed = 1;
         }
         else {
            ccsrState.button0Pressed = 0;
         }
         if(fgetc(gpio245Val) ==  (int) '1') {
            // keypad #2
            if(ccsrState.button1Pressed == 0) {
               write(pipeSoundGen[IN], &sound[keyPressed], sizeof(sound[keyPressed]));
               toggleLcdDisplayStatus();
 	       lcdEvent = EVENT_DISPLAY_STATUS;
 	       write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
            }
            ccsrState.button1Pressed = 1;
         }
         else {
            ccsrState.button1Pressed = 0;
         }
         if(fgetc(gpio246Val) ==  (int) '1') {
            // keypad #1
            if(ccsrState.button2Pressed == 0) {
               write(pipeSoundGen[IN], &sound[keyPressed], sizeof(sound[keyPressed]));
               toggleLcdDisplayMode();
 	       lcdEvent = EVENT_TOGGLE_MODE;
 	       write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
            }
            ccsrState.button2Pressed = 1;
         }
         else {
            ccsrState.button2Pressed = 0;
         }
         if(fgetc(gpio251Val) ==  (int) '1') {
            // keypad #4
            if(ccsrState.button3Pressed == 0) {
               write(pipeSoundGen[IN], &sound[keyPressed], sizeof(sound[keyPressed]));
               toggleLcdDisplayMenue();
               lcdEvent = EVENT_DISPLAY_MENUE;
               write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
            }
            ccsrState.button3Pressed = 1;
         }
         else {
            ccsrState.button3Pressed = 0;
         }
         if(fgetc(gpio248Val) ==  (int) '1') {
            // keypad #3 Toggle menue field
            if(ccsrState.button4Pressed == 0) {
 	       write(pipeSoundGen[IN], &sound[keyPressed], sizeof(sound[keyPressed]));
 	       switch(ccsrState.menueItem) {
 	          case menueItemMotorsOn:
 	             if(ccsrState.noMotors==0) {
	        	ccsrState.noMotors = 1;
 	        	say("Motors off");
	             }
	             else {
	        	ccsrState.noMotors = 0;
 	        	say("Motors on");
	             }

 	          break;
 	          case menueItemSonarOn:
 	             if(ccsrState.sonarSensorsOn==0) {
	        	ccsrState.sonarSensorsOn = 1;
 	        	say("Sonar on");
	             }
	             else {
	        	ccsrState.sonarSensorsOn = 0;
 	        	say("Sonar off");
	             }
 	          break;
 	          case menueItemNavigationOn:
	             if(ccsrState.navigationOn==0) {
	        	ccsrState.navigationOn = 1;
 	        	say("Navigation on");
	             }
	             else {
	        	ccsrState.navigationOn = 0;
 	        	say("Navigation off");
	             }
 	          break;
 	          case menueItemnvironmentalOn:
	             if(ccsrState.environmantalSensorsOn==0) {
	        	ccsrState.environmantalSensorsOn = 1;
 	        	say("Environmental on");
	             }
	             else {
	        	ccsrState.environmantalSensorsOn = 0;
 	        	say("Environmental off");
	             }
 	          break;
 	          case menueItemMotionDetectOn:
	             if(ccsrState.pidMotionDetectOn==0) {
	        	ccsrState.pidMotionDetectOn = 1;
 	        	say("Motion Detection on");
	             }
	             else {
	        	ccsrState.pidMotionDetectOn = 0;
 	        	say("Motion Detection off");
	             }
 	          break;
 	          case menueItemNoiseDetectOn :
	             if(ccsrState.noiseDetectOn==0) {
	        	ccsrState.noiseDetectOn = 1;
 	        	say("Noise Detection on");
	             }
	             else {
	        	ccsrState.noiseDetectOn = 0;
 	        	say("Noise Detection off");
	             }
 	          break;

	          }
 	       lcdEvent = EVENT_DISPLAY_MENUE;
 	       write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
	    }
            ccsrState.button4Pressed = 1;
         }
         else {
            ccsrState.button4Pressed = 0;
         }
         if(fgetc(gpio249Val) ==  (int) '1') {
            // PID motion detector
	    if(ccsrState.pidMotionDetectOn) {
 	       if(ccsrState.motionDetected == 0) {
     		  lcdEvent = EVENT_MOTION_DETECTED;
 	          write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
 	       }
 	       ccsrState.motionDetected = 1;
            }
	 }
         else {
            ccsrState.motionDetected = 0;
         }
      }
      fclose(gpio244Val);
      fclose(gpio245Val);
      fclose(gpio246Val);
      fclose(gpio247Val);
      fclose(gpio248Val);
      fclose(gpio249Val);
      fclose(gpio251Val);
   }
#endif   
}

void powerMicAmp(int on) {
   if(on) {
      fputc('1', gpio250Val);fflush(gpio250Val);
   }
   else {
      fputc('0', gpio250Val);fflush(gpio250Val);
   }
   
}

/*

               ccsrState.action = CAP_PLAYB_ACTION;
               lcdEvent = EVENT_ACTION;
               write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
               recordWave(PLAYBACK, 10000);
*/
