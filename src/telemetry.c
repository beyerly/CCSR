//############################################################################################
// 
// CCSR Robot project: http://letsmakerobots.com/robot/project/ccsr
//
// Telemetry: receives commands from different linux pipes (NLP process and/or telccsr cmd-line interface)
//            and executes these using CCSR functions.
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
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>

#include "ccsr.h"
#include "sound.h"
#include "servoCtrl.h"
#include "telemetry.h"
#include "telccsr.h"
#include "lcdDisp.h"
#include "actions.h"
#include "motor.h"
#include "facial.h"
#include "mood.h"
#include "graph.h"
#include "mapping.h"
#include "utils.h"
#include "visual.h"

extern FILE *logFile;
extern ccsrStateType ccsrState;
       ccsrStateType ccsrStateLocal;
extern int pipeLCDMsg[2];
extern int pipeSoundGen[2];
extern int pipeFacialMsg[2];
soundType sound[standardSoundsCount];


// For Bluetooth
int rfcomm;
char BTConnectionEnabled;

char eom[]  = TEL_EOM;
// FIFO names for the available cmd interfaces
char* fin[]  = {NLP_FIFO_IN,
                CCSR_FIFO_IN
                };
char* fout[] = {NLP_FIFO_OUT,
                CCSR_FIFO_OUT
		};
char dmpfile[]      = PROFILE_DUMP;
char statecsvfile[] = STATE_CSV_DUMP;
char fullstatecsvfile[] = FULL_STATE_CSV_DUMP;

// FIFO file IDs
int rfd[NUM_CMDIF];
int wfd[NUM_CMDIF];
int dfd;                 // dumpfile for sonar/ambient dumps
int stateCSVfd;          // CSV file with CCSR state dump for NLP python scripts

const char delimiters[] = " ";

// Telemetry commands
// Strings cannot exceed MAX_STRING_LEN
char *cmd_lookup[] = {"set",             // followed by set_cmd_lookup[]
 		      "dump",            // followed by dump_cmd_lookup[]
		      "setstate",        // Obsolete: can be replaced 
		      "orient",          // followed by orient_mode_lookup[]
		      "turnto",          // turnto <int> - Turn in-place to heading <int>, Navigation has to be on!
		      "say",             // say <string> - CCSR will say Full 0-terminated string  
		      "quit",            // quit - stop telemetry link, but continue running. 
		      "terminate",       // terminate - stop all processes and terminate CCSR
		      "acap",            // acap <int> - Capture audio for <int> ms, and write result out in 'voice.wav'
		                         // file. External scripts will use this for speech2text and NLP
		      "diag",            // diag - Run diagnostics function once
		      "aplay",           // Unused at this time: no audio is saved by CCSR
		      "calcomp",         // calcomp - Calibrate Compass. Turn in-place for at least 360 degress, 
		                         // measure magnitec fields and calibrate functions
		      "reset",           // Unused st this time
		      "dummy",           // dummy <int> <int> ... - Dummy command for debug.
		      "move",            // move <0=stop, 1=fwd, 2=reverse, 3=driveToTargetHeading> <time> - 
                                         // move 0 - stop
                                         // move 1/2 <time> move fwd/reverse for specified amount of time 
                                         // move 3 continuous drive forward to target heading, using evasive action
                                         // move 4 continuous drive forward to target heading, stopping if obstacle is encountered
		      "turn",            // turn <0=RIGHT, 1=LEFT, 2=degrees> <time>|<degrees> - turn left/right for specified amount of time 
                                         // or turn <degrees> relative to current heading
		      "facial",          // facial <expressionSetType> - Do facial expression using eyes and LED. For expressions see expressionset in facial.h
		      "listen",          // listen - start continuous voice recognition
		                         // listen 0 - stop continuous voice recognition
                      "obj",             // followed by obj_cmd_lookup[] - object manipulation
                      "triangulate",     // Triangulate position based on visual beacons
                      "goto"             // goto <X> <Y> drive to location (X, Y) , stopping if obstacle is encountered
		      };
// sub-commands of 'dump'
char *dump_cmd_lookup[] = {"all",        // dump all - Print selected ccsrState fields to return-fifo
			   "profile",    // dump profile - Dump the current captured values for sonar depth and ambien  
			                 // light in a CSV file PROFILE_DUMP. These values must be populated first by
					 // running a 'orient [fwd|full]' command
			   "csv",        // dump csv - Dump selected ccsrState fields to CSV file for use by NLP python scripts  
			   "disk"        // dump disk - Dump full ccsrState fields to CSV file, and camera captures to disk, used by web interface for telemetry 
			  };

// sub-commands of 'orient'
char *orient_mode_lookup[] = {"full",    // orient full - Run a full orientation: turn 360 degrees in place in 3 120 
                                         // degress steps, sweep 180 degree with pantilt in each phase, measuring sonar
					 // depth and ambient light. Values stored in ccsrState.*profile*
			      "fwd"      // orient fwd - run a single phase orientation only: 180 degree sweep of pantilt
			                 // measuring sonar-depth and ambient light, in-place, no turning.
			     };
// sub-commands of 'set'
char *set_cmd_lookup[] = {"rc",          // set rc <0,1> - turn off/on Remote control
			  "prox",        // set prox <0,1> - turn off/on proximity sensors
			  "sonar",       // set sonar <0,1> - turn off/on sonar
			  "env",         // set env <0,1> - turn off/on environmental sensors (temp, power, etc)
			  "nav",         // set nav <0,1> - turn off/on navigation (gyro, compass)
			  "pid",         // set pid <0,1> - turn off/on motion detection sensor
			  "nd",          // set nd <0,1> - turn off/on noise detection
			  "cam",         // Currently unused. set cam <0,1> - turn off/on camera capture
			  "allsens",     // set allsens <0,1> - turn off/on all sensors simultaneously
			  "speed",       // set speed <targetSpeed [-255..255]> <delta [0..256]> - set CCSR forward 
			                 // and turn-speed. This command will only finish if target speeds are met 
			  "pantilt",     // set pantilt <pan [-90..90]> <tilt [-45..45]> <speed [0..100]>- Set head (pantilt servos)
                                         // set pantilt <0=off, 1=on> - Turn on/off pantilt servo's
			                 // in specific position. Value sare degrees, 0,0 is dead ahead. 
			  "track",       // set track <0,1,2> - turn off/on opencv object tracking 0=off, 1=color thresholding, 2=shape detection 
			  "state",       // set state <int> - set ccsrState.state to state <int> 
			  "gyro",        // set gyro <0,1> - turn off/on gyro
			  "minturnspeed",// set minturnspeed - Determine minimal DC motor power needed to make CCSR
			                 // turn in place. CCSR increases engine power until gyro detects Z-axis 
					 // movement, and stores this power as the slowest speed it can use to turn 
			  "minspeed",    // set minturnspeed <int> - Set minimum motor power to move fwd/back. Currently setting fixed value
			                 // todo: determine automatically baed on linear accelerometer.
			  "maxopcurr",   // set maxopcurr <int> - Set maximum operating current in <int> mA. If CCSR
			                 // draws more than this value, the 'exceeding current limit' alarm will be 
					 // triggered  
			  "arm",         // set arm <shoulder [0..45]> <elbow [0..180]> <wrist [0..180]> <hand [0..180]> <speed [1..100]> 
			                 // set arm <0=off, 1=on> Turn arm servo's on/off 
			                 // set arm in specific position, each joint indicated in degrees. Speed
					 // indicates how fast arm moves into position. 
			  "mprescaler",  // set mprescaler <int> - Set DC motor driver prescaler 
			  "volume",      // set volume <int> - Increase speaker volume by <int> 
			  "lcddisp",     // set lcddisp <0=off,1=on> <contrast> <brightness> - Set LCD Display  
			  "tcolorvol",   // set tcolorvol <int> - Set Target Color volume, used as a threshhold to determine if CCSR
			                 // is close to target object. The larger the number, the bigger the portion of the camara image
					 // the target color must occupy before CCSR considers itself right in front of object.
                          "rgbled",      // set rgbled <R> <G> <B> - Set RGB LED color
			  "mood",         // set mood <happiness [-255..255]> <arousal [0..255]> - increment emotions
			  		 // set mood <0=off, 1=on> Turn emotions on/off 

			  "tgtclr",      // set tgtclr <string> - Set target color by name
                          "objrecogmode", // set objrecogmode <0=colorThreshold, 1=shapeDetection, 2=facialRecognition> - set object recognition mode 
			  "motors"       // set motors <0=off, 1=on> enable/disable motors 
			  };



char *onoff_cmd_lookup[] = {"0",
			   "1"
			   };

char *obj_cmd_lookup[] =  {"pickup",   // Pick up object from fixed location on floor
			   "drop",     // Put object down on fixed location on the floor
                           "give",     // Stretch arm and release object in the air
                           "analyze",  // strech arm, ask for object, when object detected, close grabber, analyse the color 
                                       // and put it down on the floor
                           "find"      // Look for and track target color, position CCSR to put object in fixed target location
                                       // pick up the object.
			   };

char *objRecogModesStrings[]   = {"Color threshold",
				  "Shape detection",
				  "Facial recognition"
				  };
char *enDisableStrings[]   = {"disabling",
			      "enabling"};

// List of selected fields in ccsrState. This is used by dump commands, to generate CSV file or pass on to telCCSR temeletry.
char *CCSRStateTemplate[] = {"state,  		      %4d, - \n",   // 0
                             "proximitySensorsOn,     %4d, - \n",   // 1
                             "sonarSensorsOn,	      %4d, - \n",   // 2
                             "environmantalSensorsOn, %4d, - \n",   // 3
                             "navigationOn,	      %4d, - \n",   // 4
                             "pidMotionDetectOn,      %4d, - \n",   // 5
                             "gyroOn,		      %4d, - \n",	// 6
                             "noiseDetectOn,	      %4d, - \n",	// 7
                             "arousal,		      %4d, - \n",	// 8
                             "happiness,	      %4d, - \n",	// 9
                             "light,                  %4d, lumen \n",	// 10
                             "irDistFrontLeft,	      %4d, - \n",	// 11
                             "irDistFrontRight,       %4d, - \n",	// 12
                             "irDistBelow,	      %4d, - \n",	// 13
                             "temperature,            %4d, degrees\n",	// 14
                             "proximity,              %4d, - \n",	// 15
                             "sonarDistFront,	      %4d, - \n",	// 16
                             "sonarDistDownFront,     %4d, - \n",	// 17
                             "compass,	              %4d, degrees\n",	// 18
                             "target,                 %4d, degrees\n",	// 19
                             "TrackObject,            %4d, - \n",	// 20
                             "Pan,                    %4d, - \n",	// 21
                             "Tilt,                   %4d, - \n",	// 22
                             "RC,                     %4d, - \n",	// 23
                             "batteryVoltage,         %4d, millivolt\n",	// 24
                             "battery,                %4d, percent\n",	// 25
                             "MaxOperatingCurr,       %4d, ampere\n",	// 26
                             "CurrentLimited,         %4d, - \n",	// 27
                             "power,                  %4d, milliwatt\n",// 28
                             "locationX,              %4f, - \n",	// 29
                             "locationY,              %4f, -"	        // 30
                             };
			     

// look up string in LUT of 'size' commands, if valid, and return cmd token
// If string not found, return a string with all valid commands
int tokenLookup(char *cmd, char **lut, char size, char valid, int wfd) {
   char i;
   char match;
   char string[MAX_STRING_LEN];
   i=-1;
   match = 0;
   if(valid) {
      while(!match && (i<size-1)) {
         i=i+1;
         match = (strcmp(cmd, lut[i]) == 0);
      }
      if(match) {
     	 return i;
      }
   }
   sprintf(string, "Unknown command, expecting: \n");
   write(wfd, string, sizeof(string));
   for(i=0;i<size;i++) {
      sprintf(string, "  %s\n", lut[i]);
      write(wfd, string, strlen(string));
   }
   write(wfd, eom, strlen(eom));
   return -1;
}   

// Open read and write fifos for cmd interface
void openCmdInterfaceFifos(int* rfd, int* wfd, char* fin, char* fout) {

   // Opening FIFOs if they don't exist. This blocks untill remote client has opened the write FIFO
   // on his side. Untill then , no telemetry running.
   printf("qq %s %s %d %d\n", fin, fout, rfd, wfd);
   if ((*rfd = open(fin, O_RDONLY)) < 0) {
      if (mkfifo(fin, S_IRWXU) != 0) {
         perror("mkfifo() error");
      }
      if ((*rfd = open(fin, O_RDONLY)) < 0) {
         perror("open() error for telemetry input FIFO");
      }
   }
   if ((*wfd = open(fout, O_WRONLY)) < 0) {
      if (mkfifo(fout, S_IRWXU) != 0) {
         perror("mkfifo() error");
      }
      if ((*rfd = open(fout, O_WRONLY)) < 0) {
         perror("open() error for telemetry input FIFO");
      }
   }  
   printf("qq %s %s %d %d\n", fin, fout, rfd, wfd);
}

// read 'eom' terminated cmd from read fifo, parse and execute, and send response back on write fifo
void ccsrParseAndExecuteCmd(int rfd, int wfd) {
   char in;
   char line[MAX_TELEMETRY_STRING_LEN];
   int i, n, q;
   char *token;
   char **splitLine;

   splitLine = malloc(MAX_ARGS*sizeof(char*));
   for(i=0;i<MAX_ARGS;i++) {
      splitLine[i] = malloc(MAX_ARG_LEN*sizeof(char));
   }
   i=0;
   n=0;

   // Read chars from FIFO untill EOM, create NULL-terminated string from    this.
   read(rfd, &in, 1);
   while(in != 42) {  // TEL_EOM
      line[i] = in;
      i=i+1;
      read(rfd, &in, 1);
   }
   line[i] = 0;
   printf("in: %s\n", line);
   
   // Split into single space-separated words
   token = strtok (line, delimiters);
   while(token != NULL) {
      strcpy(splitLine[n], token);
      n=n+1;
      token = strtok (NULL, delimiters);
   }

    ccsrExecuteCmd(splitLine, n, wfd);

   for(i=0;i<MAX_ARGS;i++) {
      free(splitLine[i]);
   }
   free(splitLine);
}

// Command interface for NLP deamon (Natural Language Processor)
// Continuously read from fifo and parse and execute
// any incoming command. This cmd_if is connected to NLPDeamon.py, a separate // // process

void *cmdInterface_NLP() {

   openCmdInterfaceFifos(&rfd[CMDIF_NLP], &wfd[CMDIF_NLP], fin[CMDIF_NLP], fout[CMDIF_NLP]); 
   while(1) {
      printf("*\n");
      ccsrParseAndExecuteCmd(rfd[CMDIF_NLP], wfd[CMDIF_NLP]);
   }
}


// Command interface for telccsr (Telemetry)
// Continuously read from fifo and parse and execute
// any incoming command. This cmd_if is connected to telccsr, a separate cmd-line
// input program run by a remote client.
void *cmdInterface_telemetry() {
   char lcdEvent;
   openCmdInterfaceFifos(&rfd[CMDIF_TELEMETRY], &wfd[CMDIF_TELEMETRY], fin[CMDIF_TELEMETRY], fout[CMDIF_TELEMETRY]); 

   ccsrState.telemetryConnected = 1;
   lcdEvent = EVENT_TELEMETRY_CONN;
   write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));

   while(ccsrState.telemetryConnected) {
      ccsrParseAndExecuteCmd(rfd[CMDIF_TELEMETRY], wfd[CMDIF_TELEMETRY]);
    }
}

void dumpCCSRState(int wfd, char** template) {
   char string[100];
   sprintf(string, template[0],ccsrState.state);write(wfd, string, strlen(string));
   sprintf(string, template[1],ccsrState.proximitySensorsOn);write(wfd, string, strlen(string));
   sprintf(string, template[2],ccsrState.sonarSensorsOn);write(wfd, string, strlen(string));
   sprintf(string, template[3],ccsrState.environmantalSensorsOn);write(wfd, string, strlen(string));
   sprintf(string, template[4],ccsrState.navigationOn);write(wfd, string, strlen(string));
   sprintf(string, template[5],ccsrState.pidMotionDetectOn);write(wfd, string, strlen(string));
   sprintf(string, template[6],ccsrState.gyroOn);write(wfd, string, strlen(string));
   sprintf(string, template[7],ccsrState.noiseDetectOn);write(wfd, string, strlen(string));
   sprintf(string, template[8],ccsrState.arousal);write(wfd, string, strlen(string));
   sprintf(string, template[9],ccsrState.happiness);write(wfd, string, strlen(string));
   sprintf(string, template[10],ccsrState.ambientLight);write(wfd, string, strlen(string));
   sprintf(string, template[11],ccsrState.irDistFrontLeft );write(wfd, string, strlen(string));
   sprintf(string, template[12],ccsrState.irDistFrontRight);write(wfd, string, strlen(string));
   sprintf(string, template[13],ccsrState.irDistBelow   );write(wfd, string, strlen(string));
   sprintf(string, template[14],ccsrState.temp    );write(wfd, string, strlen(string));
   sprintf(string, template[15],ccsrState.proximity     );write(wfd, string, strlen(string));
   sprintf(string, template[16],ccsrState.sonarDistFront  );write(wfd, string, strlen(string));
   sprintf(string, template[17],ccsrState.sonarDistDownFront);write(wfd, string, strlen(string));
   sprintf(string, template[18],ccsrState.heading); write(wfd, string, strlen(string));
   sprintf(string, template[19],ccsrState.targetHeading); write(wfd, string, strlen(string));
   sprintf(string, template[20],ccsrState.trackTargetColorOn); write(wfd, string, strlen(string));
   sprintf(string, template[21],ccsrState.pan); write(wfd, string, strlen(string));
   sprintf(string, template[22],ccsrState.tilt); write(wfd, string, strlen(string));
   sprintf(string, template[23],ccsrState.remoteControlled);write(wfd, string, strlen(string));
   sprintf(string, template[24],ccsrState.batteryVoltage);write(wfd, string, strlen(string));
   sprintf(string, template[25],ccsrState.batteryPercent);write(wfd, string, strlen(string));
   sprintf(string, template[26],ccsrState.maxOperatingCurrent);write(wfd, string, strlen(string));
   sprintf(string, template[27],ccsrState.currentLimit);write(wfd, string, strlen(string));
   sprintf(string, template[28],ccsrState.operatingCurrent*ccsrState.batteryVoltage/1000); write(wfd, string, strlen(string));
   sprintf(string, template[29],ccsrState.locationX); write(wfd, string, strlen(string));
   sprintf(string, template[30],ccsrState.locationY); write(wfd, string, strlen(string));
}

void dumpCCSRStateShort(int wfd, char** template) {
   char string[100];
   int power;
   power = ccsrState.operatingCurrent*ccsrState.batteryVoltage;
   power = power/10000;
   sprintf(string, template[8],ccsrState.arousal);write(wfd, string, strlen(string));
   sprintf(string, template[9],ccsrState.happiness);write(wfd, string, strlen(string));
   sprintf(string, template[10],ccsrState.ambientLight);write(wfd, string, strlen(string));
   sprintf(string, template[14],ccsrState.temp    );write(wfd, string, strlen(string));
   sprintf(string, template[18],ccsrState.heading); write(wfd, string, strlen(string));
   sprintf(string, template[25],ccsrState.batteryPercent);write(wfd, string, strlen(string));
   sprintf(string, template[28], power); write(wfd, string, strlen(string));
}



// Execute cmd from any input pipe (telemetry or NLP).
// cmd is array of strings
void ccsrExecuteCmd(char **splitLine, int n, int wfd) {
      int x, y;
      char lcdEvent;
      char string[MAX_STRING_LEN];
      char cmd, subCmd, subSubCmd;
      int value0, value1, value2, value3, value4;
      expressionType expr;

      // Parse command
      cmd = tokenLookup(splitLine[0], cmd_lookup, NUM_CMD, (n>0), wfd);
      switch (cmd) {
 	 case -1:
	 break;
	 case CMD_DUMP:
	    subCmd = tokenLookup(splitLine[1], dump_cmd_lookup, NUM_DUMPSUBCMD, (n>1), wfd);
 	    switch (subCmd) {
 	      case -1:
	      break;
	      case DUMPSUBCMD_ALL:
		 dumpCCSRState(wfd, CCSRStateTemplate);
	 	 write(wfd, eom, strlen(eom));
	      break;
	      case DUMPSUBCMD_CSV:
	         stateCSVfd = open(statecsvfile, O_WRONLY | O_CREAT, S_IRWXO);
		 if(stateCSVfd<0) {
		    perror("can't open CCSR state csv file\n");
		 }
		 dumpCCSRStateShort(stateCSVfd, CCSRStateTemplate);
 		 close(stateCSVfd);
 		 sprintf(string, "CCSR state written in %s\n", statecsvfile);
		 write(wfd, string, strlen(string));
 		 write(wfd, eom, strlen(eom));
	      break;
	      case DUMPSUBCMD_PROFILE:
	         dfd = open(dmpfile, O_WRONLY | O_CREAT, S_IRWXO);
		 if(dfd<0) {
		    perror("can't open dump file");
		 }
		 sprintf(string,"heading, sonarDepth, ambLight\n");
 		 write(dfd, string, strlen(string));
		 for (x=0;x<=360;x++) {
 		    if(ccsrState.profileValid[x]) {
 		       sprintf(string,"%d, %d, %d\n", x, ccsrState.sonarDistProfile[x], ccsrState.ambientLightProfile[x]);
 		       write(dfd, string, strlen(string));
 		    }
		 }
 		 close(dfd);
 		 sprintf(string, "Profile written in %s\n", dmpfile);
		 write(wfd, string, strlen(string));
 		 write(wfd, eom, strlen(eom));
	      break;
	      case DUMPSUBCMD_DISK:
	         // Capture and analyse single frame and save to disk
                 if(!analyseCameraFrame (VISUAL_CMD_CAPTURE, 1)){
                    printf("Error capturing image from camera and writing to disk|n");
                 }
                 // Save full ccsrState to CSV file on disk
                 stateCSVfd = open(fullstatecsvfile, O_WRONLY | O_CREAT, S_IRWXO);
		 if(stateCSVfd<0) {
		    perror("can't open CCSR state csv file\n");
		 }
		 dumpCCSRState(stateCSVfd, CCSRStateTemplate);
 		 close(stateCSVfd);
                 // Create sonar profile image
                 drawSonarProfileGraph();
                 // Draw current location on SVG map if known, and save map to disk
                 drawLocationInSVGMap();
                 writeSVGMap();
                 sprintf(string, "Camera images written to disk, CCSR state written in %s, SVG map written to %s\n", fullstatecsvfile, SVG_MAP_SLAM_NAME);
		 write(wfd, string, strlen(string));
 		 write(wfd, eom, strlen(eom));
	      break;
	    }
	 break;
         case CMD_OBJ:
            subCmd = tokenLookup(splitLine[1], obj_cmd_lookup, NUM_OBJSUBCMD, (n>1), wfd);
            switch (subCmd) {
            case -1:
               break;
            case OBJSUBCMD_PICKUP:
               grab0();
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
               break;
            case OBJSUBCMD_DROP:
               drop0();
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
               break;
            case OBJSUBCMD_GIVE:
               giveObjectAndFoldArm();
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
               break;
            case OBJSUBCMD_ANALYZE:
               analyzeObject();
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
               break;
            case OBJSUBCMD_FIND:
               findAndPickupObject();
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
               break;
            }
            break;
            case CMD_SET:
            subCmd = tokenLookup(splitLine[1], set_cmd_lookup, NUM_SENS_CMD, (n>1), wfd);
	    switch (subCmd) {
 	      case -1:
	      break;
	      case REMOTE:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
 	         ccsrState.remoteControlled = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break ;
	      case SENS_PROX:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.proximitySensorsOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
	      case SENS_SONAR:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.sonarSensorsOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
	      case SENS_ENV:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.environmantalSensorsOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
	      case SENS_NAV:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.navigationOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
	      case SENS_PID:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.pidMotionDetectOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
	      case SENS_GYRO:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.gyroOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
//	      case MIN_TURN_SPEED:
//	         getMinimumTurnSpeed();
//		 sprintf(string, "Command succesful\n");
// 	         write(wfd, string, strlen(string));
// 	         write(wfd, eom, strlen(eom));
//	      break;
	      case MIN_TURN_SPEED:
                 if (n>2) {
	            value0 = atoi(splitLine[2]);
	            ccsrState.minMotorTurnSpeed = value0; 
		    sprintf(string, "Command succesful\n");
 		    write(wfd, string, strlen(string));
 		    write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set minspeed <int>\n", cmd);
 		    write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }	        
	      break;
	      case MIN_SPEED:
                 if (n>2) {
	            value0 = atoi(splitLine[2]);
	            ccsrState.minMotorSpeed = value0; 
		    sprintf(string, "Command succesful\n");
 		    write(wfd, string, strlen(string));
 		    write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set minspeed <int>\n", cmd);
 		    write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }	        
	      break;
	      case SENS_NOISE:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.noiseDetectOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
	      case SENS_CAM:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.cameraCaptureOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
	      case MAX_OP_CURR:
                 if (n>2) {
	            value0 = atoi(splitLine[2]);
		    ccsrState.maxOperatingCurrent = value0;
		    sprintf(string, "Command succesful\n");
 		    write(wfd, string, strlen(string));
 		    write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set maxopcurr <current (mA)>\n", cmd);
 		    write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }	        
	      break;
              case TRACK_COLOR:
                 if (n>2) {
	            value0 = atoi(splitLine[2]);
		    ccsrState.trackTargetColorOn = value0;
	            say(enDisableStrings[value0]);
	            say("tracking");
 		    sprintf(string, "Command succesful\n");
 		    write(wfd, string, strlen(string));
 		    write(wfd, eom, strlen(eom));
		 }
                 break;
              case STATE:
                 if (n>2) {
                    value0 = atoi(splitLine[2]);
                    ccsrState.remoteControlled = 0;
                    stateChange(value0);
                    sprintf(string, "Command succesful\n");
                    write(wfd, string, strlen(string));
                    write(wfd, eom, strlen(eom));
                 }
                 else {
                    sprintf(string, "Expecting: set state <state>\n", cmd);
                    write(wfd, string, strlen(string));
                    write(wfd, eom, strlen(eom));
                 }	        
                 break;
	      case SENS_ALL:
	         subSubCmd = tokenLookup(splitLine[2], onoff_cmd_lookup, NUM_ONOFFSUBCMD, (n>2), wfd);
	         ccsrState.proximitySensorsOn = subSubCmd;
	         ccsrState.sonarSensorsOn = subSubCmd;
	         ccsrState.environmantalSensorsOn = subSubCmd;
	         ccsrState.navigationOn = subSubCmd;
	         ccsrState.pidMotionDetectOn = subSubCmd;
	         ccsrState.noiseDetectOn = subSubCmd;
	         ccsrState.cameraCaptureOn = subSubCmd;
		 sprintf(string, "Command succesful\n");
 	         write(wfd, string, strlen(string));
 	         write(wfd, eom, strlen(eom));
	      break;
	      case SPEED:
 	         if (n>3) {
	            value0 = atoi(splitLine[2]);
	            value1 = atoi(splitLine[3]);
 	            while(!speedFiltered(value0, value1)) {
 	               usleep(BRAIN_LOOP_INTERVAL);
 	            }
 	            sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set speed <target_Speed> <turn_delta>\n", cmd);
     	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	      break;
	      case PANTILT:
 	         if (n>4) {
	            value0 = atoi(splitLine[2]);
	            value1 = atoi(splitLine[3]);
	            value2 = atoi(splitLine[4]);
		    setPanTilt(value0, value1, value2);
		    sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
 	         else if (n>2) {
	            // Turning on/off pantilt servo's
		    value0 = atoi(splitLine[2]);
	            say(enDisableStrings[value0]);
	            say("head servos");
		    enablePanTilt(value0);
		    sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set pantilt <on/off [0,1]> | <pan pos> <tilt pos> <speed [1..100]>\n", cmd);
     	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	      break;
	      case ARM:
 	         if (n>6) {
	            value0 = atoi(splitLine[2]);
	            value1 = atoi(splitLine[3]);
	            value2 = atoi(splitLine[4]);
	            value3 = atoi(splitLine[5]);
	            value4 = atoi(splitLine[6]);
 	            setArm(value0, value1, value2, value3, value4);
		    sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	         else if (n>2) {
	            // Turning on/off arm servo's
		    value0 = atoi(splitLine[2]);
	            say(enDisableStrings[value0]);
	            say("robotic arm");
		    enableArm(value0);
		    sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set arm <arm pos> <elbow pos> <wrist pos> <hand pos>\n", cmd);
     	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	      break;
	      case MOTORPRESCALER:
		 if (n>2) {
	            value0 = atoi(splitLine[2]);
	            setMotorPrescalerFrequency(value0);
		    sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set mprescaler <value>\n", cmd);
     	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	      case VOLUME:
		 if (n>2) {
	            value0 = atoi(splitLine[2]);
	            value1 = ccsrState.speakerVolume + value0;
		    if (value1 > 100) {
		       // Clip at 100%
		       value1 = 100;
		    }
		    set_playback_volume(value1);
		    sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set volume <value>\n", cmd);
     	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	      break;
	      case LCDDISP:
		 if (n>4) {
	            value0 = atoi(splitLine[2]);
	            value1 = atoi(splitLine[3]);
	            value2 = atoi(splitLine[4]);
		    lcdDisplayPower(value0);
		    lcdDisplayConfig(value1, value2);
		    sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set lcddisp <0,1> <contrast> <brightness> \n", cmd);
     	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	      break;
	      case TARGET_COLOR_VOLUME:
		 if (n>2) {
	            value0 = atoi(splitLine[2]);
	            setTargetColorVolume(value0);
		    sprintf(string, "Command succesful\n");
 	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	         else {
 	            sprintf(string, "Expecting: set tcolorvol <value>\n", cmd);
     	            write(wfd, string, strlen(string));
 	            write(wfd, eom, strlen(eom));
	         }
	      break;
         case RGBLED:
            if (n>5) {
               value0 = atoi(splitLine[2]);
               value1 = atoi(splitLine[3]);
               value2 = atoi(splitLine[4]);
               value3 = atoi(splitLine[5]);
               setRGBLED(value0, value1, value2, value3);
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            else {
               sprintf(string, "Expecting: set rgbled <R [0..255]> <B [0..255]> <B [0..255]> <speed [0..100]>\n", cmd);
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            break;
         case MOOD:
            if (n>3) {
               value0 = atoi(splitLine[2]);
               value1 = atoi(splitLine[3]);
               setMood(value0, value1);
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            else if(n>2){
               value0 = atoi(splitLine[2]);
	       say(enDisableStrings[value0]);
	       say("emotions");
	       ccsrState.showEmotion = value0;
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            else {
               sprintf(string, "Expecting: set mood <happiness> <arousal>\n", cmd);
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
	      break;
         case TGT_COLOR:
            if (n>2) {
	       say("target color set to");
 	       say(splitLine[2]);
               setTargetColorRangeByName(splitLine[2]);
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            else {
               sprintf(string, "Expecting: set tgtClr <name>\n", cmd);
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
         case OBJ_RECOG_MODE:
            if (n>2) {
 	       value0 = atoi(splitLine[2]);
	       ccsrState.objectRecognitionMode = value0;
	       say("Setting object recognition mode to"); 
	       say(objRecogModesStrings[value0]);
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            else {
               sprintf(string, "Expecting: set \n", cmd);
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            break;
         case MOTORS:
            if (n>2) {
 	       value0 = atoi(splitLine[2]);
	       say(enDisableStrings[value0]);
	       say("motors");
               sprintf(string, "Command succesful\n");
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            else {
               sprintf(string, "Expecting: set motors <name>\n", cmd);
               write(wfd, string, strlen(string));
               write(wfd, eom, strlen(eom));
            }
            break;
            }
	 break;
	 case CMD_TURN_TO:
            if (n>1) {
	       value0 = atoi(splitLine[1]);
	       ccsrState.targetHeading = value0;
     	       turnToTargetHeading(NOSCAN);
 	       sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else {
 	       sprintf(string, "Expecting: turnto <heading> \n", cmd);
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
         case CMD_TRIANGULATE:
            if(triangulatePosition()){
               sprintf(string, "Command succesful\n");
            }
            else{
               sprintf(string, "Command unsuccesful\n");
            }
            write(wfd, string, strlen(string));
            write(wfd, eom, strlen(eom));
            break;
         case CMD_CALIBRATE_COMPASS:
	       calibrateCompass();
	       sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	 break;
	 case CMD_RESET:
	       ccsrState.currentLimit = 0;
	       //ccsrState.state = SM_REMOTE_CONTROLLED;
	       
	       sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	 break;
	 case CMD_ORIENTATION:
            subCmd = tokenLookup(splitLine[1], orient_mode_lookup, NUM_ORIENT_MODES, (n>1), wfd);
 	       if(subCmd>=0) {
	       orientation(subCmd);
 	       sprintf(string, "Deepest sonar depth: %d\n", deepestSonarDepthHeading());
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
	 case CMD_SAY:
 	    sprintf(string, "");
            for (y=1;y<n;y++) {
	       strcat(string, splitLine[y]);
	       strcat(string, " ");
	    }   
	    say(string);
	    sprintf(string, "Command succesful\n");
 	    write(wfd, string, strlen(string));
 	    write(wfd, eom, strlen(eom));
	 break;
	 case CMD_QUIT:
 	    sprintf(string, "Terminatign Connection...\n");
 	    write(wfd, string, strlen(string));
 	    write(wfd, eom, strlen(eom));
 	    ccsrState.telemetryConnected = 0;
 	    //lcdEvent = EVENT_TELEMETRY_DISCONN;
 	    //write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
	 break;
  	 case CMD_TERM:
 	    sprintf(string, "Terminatign CCSR...\n");
 	    write(wfd, string, strlen(string));
 	    write(wfd, eom, strlen(eom));
            ccsrTerminate();
	 break;
	 case CMD_AUDIO_CAPTURE:
            if (n>1) {
	       value0 = atoi(splitLine[1]);
               write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[singleA]));
 	       recordWave(VOICE_RECOGNITION, value0);
	       sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else {
 	       sprintf(string, "Expecting: acap <sample_lenght> \n", cmd);
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
	 case CMD_AUDIO_PLAYBACK:
            // Not used for now
	    if (n>1) {
	       value0 = atoi(splitLine[1]);
               playAudioMemory(value0);
	       sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else {
 	       sprintf(string, "Expecting: aplay <recording_number> \n", cmd);
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
	 case CMD_DIAGNOSTICS:
 	    say("Running Diagnostics");
	    diagnostics();
	    sprintf(string, "Command succesful\n");
 	    write(wfd, string, strlen(string));
 	    write(wfd, eom, strlen(eom));
	 break;
	 case CMD_MOVE:
	    if (n>1) {
	       value0 = atoi(splitLine[1]);
	       if(value0==0){
                  // Stop
                  ccsrState.driveToTargetHeading=0;
               }
               else if(value0 == 3){
                  // Continuous drive to target heading, using evasive action
                  ccsrState.evasiveAction=1;
                  ccsrState.driveToTargetHeading=1;
               }
               else if(value0 == 4){
                  // Continuous drive to target heading, stop if obstacle encountered
                  ccsrState.evasiveAction=0;
                  ccsrState.driveToTargetHeading=1;
               }
               else if(value0 == 5){
                  if (n>3){ 
                     value1 = atoi(splitLine[2]);
                     value2 = atoi(splitLine[3]);

                  }
                  else{
                     sprintf(string, "Expecting move 5 X Y\n");
                     write(wfd, string, strlen(string));
                  }
               }
               else{
                  // Drive right/left for specified amount of time (in ms)
                  value1 = atoi(splitLine[2]);
                  driveAtMinPower(value0, value1); 
               }
               sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else {
 	       sprintf(string, "Expecting: move <0=stop, 1=fwd, 2=reverse, 3=driveToTargetHeading>  <time> \n", cmd);
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
	 case CMD_GOTO:
 	    if (n>2) {
	       value0 = atoi(splitLine[1]);
	       value1 = atoi(splitLine[2]);
               gotoLocation(value0, value1);
               sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else {
 	       sprintf(string, "Expecting: goto X Y \n", cmd);
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
	 case CMD_TURN:
 	    if (n>2) {
	       value0 = atoi(splitLine[1]);
	       value1 = atoi(splitLine[2]);
 	       if (value0 < 2) {
                  // Turn left or right for specified amount of time
                  turnAtMinPowerInPlace(value0, value1); 
               }
               else {
                  // Turn amount of degrees relative to current compass heading
	          ccsrState.targetHeading = addAngleToHeading(value1);

     	          turnToTargetHeading(NOSCAN);
               }
               sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else {
 	       sprintf(string, "Expecting: turn <0=RIGHT, 1=LEFT> <time> \n", cmd);
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
	 case CMD_FACIAL:
 	    if (n==2) {
	       value0 = atoi(splitLine[1]);
               expr.type = value0;
	       write(pipeFacialMsg[IN], &expr,sizeof(expr));
               sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
 	    else if (n>2) {
	       value0 = atoi(splitLine[1]);
	       value1 = atoi(splitLine[2]);
               expr.type = value0;
               expr.length = value1;
	       write(pipeFacialMsg[IN], &expr,sizeof(expr));
               sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else {
 	       sprintf(string, "Expecting: facial <type> <lenght> \n", cmd);
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
	 case CMD_LISTEN:
	    if (n==1) {
               ccsrState.continuousVoiceRecognitionOn = 1;
               sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else if (n>1) {
	       value0 = atoi(splitLine[1]);
               ccsrState.continuousVoiceRecognitionOn = value0;
               printf ("VR: %d\n", ccsrState.continuousVoiceRecognitionOn);
	       sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	    else {
 	       sprintf(string, "Expecting: listen | listen <0> \n", cmd);
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	    }
	 break;
	 case CMD_DUMMY:
            // print color of roi in cameraview to stdout. Used to determine beacon color.
            ccsrState.analyzeObject = 1;
            while(ccsrState.analyzeObject){
               usleep(1000);
            }
            sprintf(string, "Command succesful\n");
 	       write(wfd, string, strlen(string));
 	       write(wfd, eom, strlen(eom));
	 break;
     }
   }





 
