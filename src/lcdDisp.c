

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include "ccsr.h"
#include "lcdDisp.h"
#include "utils.h"

#ifndef I2C_SLAVE
#define I2C_SLAVE 0
#endif

extern FILE *logFile;
extern ccsrStateType ccsrState;
extern int i2cbus;
extern pthread_mutex_t semI2c;
extern int pipeLCDMsg[2];
char lcdEvent;

//                    0               16
char *sm_lookup[] = {"SM_RESET        ",    // 0
 		     "SM_DIAGNOSTICS  ",    // 1
		     "SM_ORIENTATION  ",    // 2
		     "SM_DRIVE_TO_TGT ",    // 3
 		     "SM_POSITION_PKUP",    // 4
		     "SM_PICKUP_OBJ   ",    // 5
		     "SM_RTRN_TO_STRT ",    // 6
		     "SM_EXPLORE      ",    // 7
		     "SM_OBSERVE      ",    // 8
		     "Awaiting command",    // 9
		     "SM_TRN_LCKD_OBJ ",    // 10
		     "SM_TRK_AND_FLW  ",    // 11
                     "SM_SLEEP        "     // 12 
		      };
char *sm_espeak_lookup[] = {"Reset",
 			    "Running diagnostics",
			    "Orientation",
			    "Heading for target",
 			    "Position for pickup",
			    "Picking up object",
			    "Returning to start",
			    "Exploring       ",
			    "Observing",
			    "Awaiting command",
		            "xx ",
		            "xx  ",
			    "Goodnight!"
			     };

//                        0              16
char *action_lookup[] = {"Evasive Action ",  // 0
 			 "Orientation    ",  // 1
			 "Paused         ",  // 2
			 "Turn to Target ",  // 3
			 "Capt & Playb   ",  // 4
			 "Idle           ",  // 5
			 "Analyzing Obj  "   // 6
			 };

char majorMsg[17];
char minorMsg[17];
char majorMsgHasArg;
char minorMsgHasArg;
int* majorMsgArgPtr;
int* minorMsgArgPtr;

char eventMsg[17];
char actionMsg[17];

void lcdDisplayInit() {
//   lcdDisplayConfig(value1, value2);
}

void lcdDisplayMajorMsg(char *s) {

   unsigned char buffer[2];

   int size;
   size = strlen(s);

   buffer[0] = LCDPREFIX;
   buffer[1] = LCD_CURSORHOME;
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, LCD_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,2 ) !=2 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, s, size) != size) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   pthread_mutex_unlock(&semI2c);
} 

void lcdDisplayMinorMsg(char *s) {

   unsigned char buffer[3];

   int size;
   size = strlen(s);

   buffer[0] = LCDPREFIX;
   buffer[1] = LCD_SETCURSOR;
   buffer[2] = LCD_NEXTLINE;
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, LCD_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,3 ) !=3 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, s, size) != size) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   pthread_mutex_unlock(&semI2c);
} 


// Set LCD display contrast and brightness
// contrast = [1..50]
// brightness = 1 (off) .. 8 (brightest)
void lcdDisplayConfig(char contrast , char brightness) {
   unsigned char buffer[3];

   buffer[0] = LCDPREFIX;
   buffer[1] = LCD_CONTRAST;
   buffer[2] = contrast;
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, LCD_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,3 ) !=3 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   buffer[1] = LCD_BRIGHTNESS;
   buffer[2] = brightness;
   if(write(i2cbus, buffer,3 ) !=3 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
} 

// Clear LCD screen
void lcdDisplayClear() {

   unsigned char buffer[2];

   buffer[0] = LCDPREFIX;
   buffer[1] = LCD_CLEARSCREEN;

   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, LCD_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer, 2) !=2 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
} 

// Turn on/off LCD display (text only, not backlight).
// on=1: on, on=0: off 
void lcdDisplayPower(int on) {

   unsigned char buffer[2];

   buffer[0] = LCDPREFIX;
   if (on) {
      buffer[1] = LCD_DISPLAY_ON;
   }
   else {
      buffer[1] = LCD_DISPLAY_OFF;
   }
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, LCD_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer, 2) !=2 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
} 


void *lcdManager() {

   char string[16];
   char value[16];

   char event;
   int result;

   logMsg(logFile, "Starting LCD Display Manager", LOG); 

   majorMsgHasArg = 0;
   minorMsgHasArg = 0;
   ccsrState.statusField=fieldNone;
   ccsrState.minorMsgMode = SHOW_ACTION;
   strcpy(eventMsg, "No event        ");
   strcpy(actionMsg, "No action      ");
   while(1) {
      result = read (pipeLCDMsg[OUT],&event,sizeof(event));
      switch(event) {
 	 case EVENT_LINUX_BOOTED:
	    strcpy (majorMsg, "Linux booted    ");
	    strcpy (minorMsg, "Awaiting start");
//	    say(majorMsg);
//	    say(minorMsg);
 	 break;
 	 case EVENT_HOLD_TO_TERMINATE:
	    strcpy (majorMsg, "Hold 3 sec to   ");
	    strcpy (minorMsg, "Terminate...    ");
 	 break;
 	 case EVENT_TERMINATE:
	    strcpy (majorMsg, "Terminated      ");
	    strcpy (minorMsg, "CCSR Squirl 1   ");
	    say("Sqruirrel 1 terminated, goodbye!");
 	 break;
 	 case EVENT_CCSR_STARTED:
	    strcpy (majorMsg, "Started...      ");
	    strcpy (minorMsg, "CCSR Squirl 1   ");
	    // say("Hi! I am Squirrel one. How are you?");
 	 break;
 	 case EVENT_SM_STATE_CHANGE:
	    strcpy (majorMsg, sm_lookup[ccsrState.state]);
//	    say(sm_espeak_lookup[ccsrState.state]);
 	 break;
 	 case EVENT_MOTION_DETECTED:
	    strcpy (eventMsg, "Motion detected ");
   	    say(eventMsg);
	    if(ccsrState.minorMsgMode == SHOW_EVENT) {
	       strcpy (minorMsg, eventMsg);
	    }
 	 break;
 	 case EVENT_NOISE_DETECTED:
	    strcpy (eventMsg, "Noise detected  ");
   	    say(eventMsg);
	    if(ccsrState.minorMsgMode == SHOW_EVENT) {
	       strcpy (minorMsg, eventMsg);
	    }
 	 break;
 	 case EVENT_CURR_LIMIT:
	    strcpy (eventMsg, "Exc Curr Limit  ");
   	    say("Exceeding current limit");
	    if(ccsrState.minorMsgMode == SHOW_EVENT) {
	       strcpy (minorMsg, eventMsg);
	    }
 	 break;
 	 case EVENT_TRACKING_OBJECT:
	    strcpy (eventMsg, "Tracking Object ");
   	    say("Found Target Object, tracking!");
	    if(ccsrState.minorMsgMode == SHOW_EVENT) {
	       strcpy (minorMsg, eventMsg);
	    }
 	 break;
 	 case EVENT_TARGET_LOCKED:
	    strcpy (eventMsg, "Object Locked");
   	    say("Locked on target Object");
	    if(ccsrState.minorMsgMode == SHOW_EVENT) {
	       strcpy (minorMsg, eventMsg);
	    }
 	 break;
 	 case EVENT_LOW_BATT:
	    strcpy (eventMsg, "Low Batt        ");
   	    say("Low battery");
	    if(ccsrState.minorMsgMode == SHOW_EVENT) {
	       strcpy (minorMsg, eventMsg);
	    }
 	 break;
 	 case EVENT_TELEMETRY_CONN:
	    strcpy (eventMsg, "BT connected ");
//   	    say("Telemetry connected");
	    if(ccsrState.minorMsgMode == SHOW_EVENT) {
	       strcpy (minorMsg, eventMsg);
	    }
 	 break;
 	 case EVENT_ACTION:
	    strcpy (actionMsg, action_lookup[ccsrState.action]);
	    if(ccsrState.minorMsgMode == SHOW_ACTION) {
	       strcpy (minorMsg, actionMsg);
	    }
 	 break;
 	 case EVENT_TOGGLE_MODE:
	    switch(ccsrState.minorMsgMode) {
	       case SHOW_ACTION:
	          strcpy (minorMsg, actionMsg);
	       break;
	       case SHOW_EVENT:
	          strcpy (minorMsg, eventMsg);
	       break;
	       case SHOW_MENUE:
 		  lcdEvent = EVENT_DISPLAY_MENUE;
 		  write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
	       break;
 	       case SHOW_STATUS:
 		  lcdEvent = EVENT_DISPLAY_STATUS;
 		  write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
	       break;
	    }
 	 break;
 	 case EVENT_DISPLAY_STATUS:
	    // 11 char for minor Msg, space + 4 char for value = 16
            ccsrState.minorMsgMode = SHOW_STATUS; 
	    ccsrState.continuousLCDRefresh = 1;

	    minorMsgHasArg = 1;
	    switch(ccsrState.statusField) {
 	       case fieldBattery:
 	          strcpy (minorMsg, "Batt Level:");
 	          minorMsgArgPtr = &ccsrState.batteryPercent;
                  minorMsgHasArg = 1;
 	       break;
 	       case fieldHeading:
 	          strcpy (minorMsg, "Heading:   ");
 	          minorMsgArgPtr = &ccsrState.heading;
                  minorMsgHasArg = 1;
 	       break;
 	       case fieldtargetHeading:
 	          strcpy (minorMsg, "TgtHeading:");
 	          minorMsgArgPtr = &ccsrState.targetHeading;
                  minorMsgHasArg = 1;
 	       break;
 	       case fieldTemp:
 	          strcpy (minorMsg, "Temp:      ");
 	          minorMsgArgPtr = &ccsrState.temp;
                  minorMsgHasArg = 1;
 	       break;
               case fieldSonarDistFront:
 	          strcpy (minorMsg, "Sonar Dist ");
 	          minorMsgArgPtr = &ccsrState.sonarDistFront;
                  minorMsgHasArg = 1;
 	       break;
               case fieldAmbLight:
 	          strcpy (minorMsg, "Amb. Light ");
 	          minorMsgArgPtr = &ccsrState.ambientLight;
                  minorMsgHasArg = 1;
 	       break;
               case fieldFear:
 	          strcpy (minorMsg, "Emot: fear ");
 	          minorMsgArgPtr = &ccsrState.fear;
                  minorMsgHasArg = 1;
 	       break;

  	       case fieldNone:
   	       strcpy (minorMsg, action_lookup[ccsrState.action]);
               minorMsgHasArg = 0;
	       break;
	    }
 	 break;
 	 case EVENT_DISPLAY_MENUE:
	    // 11 char for minor Msg, space + 4 char for value = 16
            ccsrState.minorMsgMode = SHOW_MENUE; 
	    switch(ccsrState.menueItem) {
 	       case menueItemMotorsOn:
	          if(ccsrState.noMotors) {
 	             strcpy (minorMsg, "Motors Off      ");
		  }
		  else {
 	             strcpy (minorMsg, "Motors On       ");
		  }
 	       break;
 	       case menueItemSonarOn:
	          if(ccsrState.noMotors) {
 	             strcpy (minorMsg, "Sonar On        ");
		  }
		  else {
 	             strcpy (minorMsg, "Sonar Off       ");
		  }
 	       break;
 	       case menueItemNavigationOn:
	          if(ccsrState.navigationOn) {
 	             strcpy (minorMsg, "Navigation On   ");
		  }
		  else {
 	             strcpy (minorMsg, "Navigation Off  ");
		  }
 	       break;
 	       case menueItemnvironmentalOn:
	          if(ccsrState.environmantalSensorsOn) {
 	             strcpy (minorMsg, "Env.sensors On  ");
		  }
		  else {
 	             strcpy (minorMsg, "Env.sensors Off  ");
		  }
 	       break;
 	       case menueItemMotionDetectOn:
	          if(ccsrState.pidMotionDetectOn) {
 	             strcpy (minorMsg, "Motion Det. On   ");
		  }
		  else {
 	             strcpy (minorMsg, "Motion Det. Off  ");
		  }
 	       break;
 	       case menueItemNoiseDetectOn :
	          if(ccsrState.noiseDetectOn) {
 	             strcpy (minorMsg, "Noise Det. On    ");
		  }
		  else {
 	             strcpy (minorMsg, "Noise Det. Off   ");
		  }
 	       break;
	    }
 	 break;

      }
      lcdDisplayRefresh();
   }
} 


void *lcdRefresh() {

   char string[17];

   logMsg(logFile, "Starting LCD refresh", LOG); 
   while(1) {
      if(ccsrState.continuousLCDRefresh) {
         lcdDisplayRefresh();
      }
      usleep(LCD_REFRESH_PERIOD);
   }
} 

void lcdDisplayRefresh() {

   char string[17];

   if(majorMsgHasArg) {
      sprintf(string, "%s %4d", majorMsg, *majorMsgArgPtr);
      lcdDisplayMajorMsg(string);
   }
   else {
      lcdDisplayMajorMsg(majorMsg);
   }
   if(minorMsgHasArg) {
      sprintf(string, "%s %4d", minorMsg, *minorMsgArgPtr);
      lcdDisplayMinorMsg(string);
   }
   else {
      lcdDisplayMinorMsg(minorMsg);
   }
} 



void toggleLcdDisplayStatus() {
   if(ccsrState.statusField == numFields - 1) {
      ccsrState.statusField = 0;
   }
   else {
      ccsrState.statusField = ccsrState.statusField + 1;
   }
}
void toggleLcdDisplayMenue() {
   if(ccsrState.menueItem == numMenueItems - 1) {
      ccsrState.menueItem = 0;
   }
   else {
      ccsrState.menueItem = ccsrState.menueItem + 1;
   }
}
   
void toggleLcdDisplayMode() {
   if(ccsrState.minorMsgMode == NUM_SHOW_MODES - 1) {
      ccsrState.minorMsgMode = 0;
   }
   else {
      ccsrState.minorMsgMode = ccsrState.minorMsgMode + 1;
   }
   if(ccsrState.minorMsgMode==SHOW_STATUS) {
      ccsrState.continuousLCDRefresh = 1;
   }
   else {
      ccsrState.continuousLCDRefresh = 0;
   }
}


