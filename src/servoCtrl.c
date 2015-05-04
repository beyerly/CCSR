//############################################################################################
// 
// CCSR Robot project: http://letsmakerobots.com/robot/project/ccsr
//
// Servo Control: Manages all servo activity; pan/tilt of head, and arm. Also provides pthread 
//                for object tracking
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
#include <linux/i2c-dev.h>


#include "ccsr.h"
#include "sound.h"
#include "lcdDisp.h"
#include "servoCtrl.h"

#ifndef I2C_SLAVE
#define I2C_SLAVE 0
#endif


int x;
extern FILE *logFile;
extern ccsrStateType ccsrState;
extern int pipeLCDMsg[2];
char lcdEvent;
extern soundType sound[NUM_SOUNDS];
extern int pipeSoundGen[2];
extern int i2cbus;
extern pthread_mutex_t semI2c;




// Initialization of PCA9685 servo controller, called once at boot
void configServoControl() {
   char buffer[4];
   char  config_reg;
   unsigned char conv[9];

   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, PCA9685_ADDR)) {
      logMsg(logFile, "Can't set PCA9685_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);

   config_reg = 101;  // 60Hz refresh for Servo's
   buffer[0] = (char) PCA9685_REG_PRESCALE;
   buffer[1] = (char) config_reg  & 0xFF;
   if(write(i2cbus, buffer, 2) != 2) {
      logMsg(logFile, "Unsuccessful cmd write to PCA9685", ERROR);	  
   }
   usleep(I2C_DELAY);
   
   
   config_reg = (MODE0_NORMAL << MODE0_SLEEP_offset) |  // wake up
                (MODE1_AI << MODE1_AI_offset);          // Auto increment addresses


   buffer[0] = (char) PCA9685_REG_MODE0;
   buffer[1] = (char) config_reg  & 0xFF;
   if(write(i2cbus, buffer, 2) != 2) {
      logMsg(logFile, "Unsuccessful cmd write to PCA9685", ERROR);	  
   }


   usleep(I2C_DELAY);

   pthread_mutex_unlock(&semI2c);
   
   // Set servo's in neutral position
   ccsrState.pan = 1;
   ccsrState.tilt = 1;
   setPanTilt(10, 10, 100);     // Dead ahead
   setPanTilt(0, 0, 30);     // Dead ahead
   setArm(45, 5, 0, 0, 100);  // Folded
   enableArm(0);
   enablePanTilt(0);

} 


// Set pan and tilt of (camera/sonar) head to specific values. Head moves at <speed>
// pan and tilt values expressed in degrees, pan ranges 180 deg, and tilt 90 deg: 
// pan [-90..90], tilt [-45..45]
// Speed expressed as percentage of max: speed[0..100]
// (pan, tilt) = (0, 0) is dead ahead, equal to ccsrState.heading
void setPanTilt(int pan, int tilt, int speed) {
   unsigned char conv[9];
   int n,m;
   int panPW, tiltPW;
   int panPWInc, tiltPWInc;
   int delay;

   int panShift, tiltShift;
   
   
   // shift pan range -90..90 deg. to 0..180  
   // shift tilt range -45..45 deg. to 0..90  
   panShift = pan + 90;
   tiltShift = tilt + 45;

   
   // speed = 0, 1 sec per step
   // speed = 100, 0 sec per step 
   delay = 10000 - speed * 100;

   
   // 100% = SERVOMAX, 0% = SERVOMIN
   panPW  = PAN_SERVO_MIN + panShift*(PAN_SERVO_MAX-PAN_SERVO_MIN)/PAN_ANGLE_RANGE;
   tiltPW = TILT_SERVO_MIN + tiltShift*(TILT_SERVO_MAX-TILT_SERVO_MIN)/TILT_ANGLE_RANGE;


   if(speed == 100) {
      panPWInc = panPW - ccsrState.panPulseWidth;
      tiltPWInc = tiltPW - ccsrState.tiltPulseWidth;
   }
   else {
   if(ccsrState.panPulseWidth > panPW) {
      panPWInc = -PW_STEP;
   }
   else {
      panPWInc = PW_STEP;
   }
   if(ccsrState.tiltPulseWidth > tiltPW) {
      tiltPWInc = -PW_STEP;
   }
   else {
      tiltPWInc = PW_STEP;
   }
   }
   while((ccsrState.panPulseWidth!=panPW) ||
         (ccsrState.tiltPulseWidth!=tiltPW)) {
      ccsrState.panPulseWidth = ccsrState.panPulseWidth + panPWInc;
      ccsrState.tiltPulseWidth = ccsrState.tiltPulseWidth + tiltPWInc;
      if(panPWInc>0) {
         if(ccsrState.panPulseWidth>=panPW) {
            ccsrState.panPulseWidth = panPW;
         }
      }
      else {
         if(ccsrState.panPulseWidth<panPW) {
            ccsrState.panPulseWidth = panPW;
         }
      }
      if(tiltPWInc>0) {
         if(ccsrState.tiltPulseWidth>=tiltPW) {
            ccsrState.tiltPulseWidth = tiltPW;
         }
      }
      else {
         if(ccsrState.tiltPulseWidth<tiltPW) {
            ccsrState.tiltPulseWidth = tiltPW;
         }
      }
      conv[0] = PCA9685_REG_LED0_ON_L;
      conv[1] = 0x00;  // On
      conv[2] = 0x00;
      conv[3] = (unsigned char) ccsrState.panPulseWidth & 0xFF;  
      conv[4] = (unsigned char) (ccsrState.panPulseWidth >> 8 ) & 0x0F;
      conv[5] = 0x00;  // On
      conv[6] = 0x00;
      conv[7] = (unsigned char) ccsrState.tiltPulseWidth & 0xFF;  
      conv[8] = (unsigned char) (ccsrState.tiltPulseWidth >> 8 ) & 0x0F;
      pthread_mutex_lock(&semI2c);
 
      if(ioctl(i2cbus, I2C_SLAVE, PCA9685_ADDR)) {
         logMsg(logFile, "Can't set PCA9685_ADDR I2C address", ERROR);
      }
      usleep(I2C_DELAY);
 
 
      if(write(i2cbus, conv, 9) != 9) {
         logMsg(logFile, "Unsuccessful cmd write to PCA9685", ERROR);
      }
      usleep(I2C_DELAY);
      pthread_mutex_unlock(&semI2c);
      usleep(delay);
   }
   ccsrState.pan = pan;
   ccsrState.tilt = tilt;
   
} 


void setRGBLED(int R, int G, int B, int speed) {
   unsigned char conv[13];
   int n,m;
   int RPW, GPW, BPW;
   int RPWInc, GPWInc, BPWInc;
   int delay;

   
   // speed = 0, 1 sec per step
   // speed = 100, 0 sec per step 
   delay = 10000 - speed * 100;

   
   // Common annode LED, so 100% 'on' pulse width is fully off
   // Calculate such that if R=R_MAX, pulse width is 0 (fully on) 
   RPW  = R_MAX - R*(R_MAX-R_MIN)/R_RANGE;
   GPW  = G_MAX - G*(G_MAX-G_MIN)/G_RANGE;
   BPW  = B_MAX - B*(B_MAX-B_MIN)/B_RANGE;

   if(speed == 100) {
      RPWInc = RPW - ccsrState.RPulseWidth;
      GPWInc = GPW - ccsrState.GPulseWidth;
      BPWInc = BPW - ccsrState.BPulseWidth;
   }
   else {
      if(ccsrState.RPulseWidth > RPW) {
         RPWInc = -LEDPW_STEP;
      }
      else {
         RPWInc = LEDPW_STEP;
      }
      if(ccsrState.GPulseWidth > GPW) {
         GPWInc = -LEDPW_STEP;
      }
      else {
         GPWInc = LEDPW_STEP;
      }
      if(ccsrState.BPulseWidth > BPW) {
         BPWInc = -LEDPW_STEP;
      }
      else {
         BPWInc = LEDPW_STEP;
      }
   }
   while((ccsrState.RPulseWidth!=RPW) ||
         (ccsrState.GPulseWidth!=GPW) ||
	 (ccsrState.BPulseWidth!=BPW)) {
      ccsrState.RPulseWidth = ccsrState.RPulseWidth + RPWInc;
      ccsrState.GPulseWidth = ccsrState.GPulseWidth + GPWInc;
      ccsrState.BPulseWidth = ccsrState.BPulseWidth + BPWInc;
      if(RPWInc>0) {
         if(ccsrState.RPulseWidth>=RPW) {
            ccsrState.RPulseWidth = RPW;
         }
      }
      else {
         if(ccsrState.RPulseWidth<RPW) {
            ccsrState.RPulseWidth = RPW;
         }
      }
      if(GPWInc>0) {
         if(ccsrState.GPulseWidth>=GPW) {
            ccsrState.GPulseWidth = GPW;
         }
      }
      else {
         if(ccsrState.GPulseWidth<GPW) {
            ccsrState.GPulseWidth = GPW;
         }
      }
      if(BPWInc>0) {
         if(ccsrState.BPulseWidth>=BPW) {
            ccsrState.BPulseWidth = BPW;
         }
      }
      else {
         if(ccsrState.BPulseWidth<BPW) {
            ccsrState.BPulseWidth = BPW;
         }
      }
      conv[0] = PCA9685_REG_LED6_ON_L;
      conv[1] = 0x00;  // On
      conv[2] = 0x00;
      conv[3] = (unsigned char) ccsrState.RPulseWidth & 0xFF;  
      conv[4] = (unsigned char) (ccsrState.RPulseWidth >> 8 ) & 0x0F;
      conv[5] = 0x00;  // On
      conv[6] = 0x00;
      conv[7] = (unsigned char) ccsrState.GPulseWidth & 0xFF;  
      conv[8] = (unsigned char) (ccsrState.GPulseWidth >> 8 ) & 0x0F;
      conv[9] = 0x00;  // On
      conv[10] = 0x00;
      conv[11] = (unsigned char) ccsrState.BPulseWidth & 0xFF;  
      conv[12] = (unsigned char) (ccsrState.BPulseWidth >> 8 ) & 0x0F;

      pthread_mutex_lock(&semI2c);
 
      if(ioctl(i2cbus, I2C_SLAVE, PCA9685_ADDR)) {
         logMsg(logFile, "Can't set PCA9685_ADDR I2C address", ERROR);
      }
      usleep(I2C_DELAY);
 
 
      if(write(i2cbus, conv, 13) != 13) {
         logMsg(logFile, "Unsuccessful cmd write to PCA9685", ERROR);
      }
      usleep(I2C_DELAY);
      pthread_mutex_unlock(&semI2c);
      usleep(delay);
   }
   
} 


// Set pan and tilt of (camera/sonar) head to specific values. Head moves at <speed>
void enablePanTilt(int on) {
   unsigned char conv[9];
      
   if(on==1){
      printf("Resuming current pantilt servo positions\n");
      conv[0] = PCA9685_REG_LED0_ON_L;
      conv[1] = 0x00;  // On
      conv[2] = 0x00;
      conv[3] = (unsigned char) ccsrState.panPulseWidth & 0xFF;  
      conv[4] = (unsigned char) (ccsrState.panPulseWidth >> 8 ) & 0x0F;
      conv[5] = 0x00;  // On
      conv[6] = 0x00;
      conv[7] = (unsigned char) ccsrState.tiltPulseWidth & 0xFF;  
      conv[8] = (unsigned char) (ccsrState.tiltPulseWidth >> 8 ) & 0x0F;
   }
   else {
      printf("Turning off pantilt servo's\n");
      conv[0] = PCA9685_REG_LED0_ON_L;
      conv[1] = 0x00;  // On
      conv[2] = 0x00;
      conv[3] = 0x00;  
      conv[4] = (unsigned char) (LED_FULL_OFF << LED_FULL_OFF_offset) & 0xFF;
      conv[5] = 0x00;  // On
      conv[6] = 0x00;
      conv[7] = 0x00;  
      conv[8] = (unsigned char) (LED_FULL_OFF << LED_FULL_OFF_offset) & 0xFF;
   }
   
   pthread_mutex_lock(&semI2c);
 
   if(ioctl(i2cbus, I2C_SLAVE, PCA9685_ADDR)) {
      logMsg(logFile, "Can't set PCA9685_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
 
   if(write(i2cbus, conv, 9) != 9) {
      logMsg(logFile, "Unsuccessful cmd write to PCA9685", ERROR);
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
} 


// Set pan and tilt of (camera/sonar) head to specific values. Head moves at <speed>
void enableArm(int on) {
   unsigned char conv[17];
      
   if(on==1){
      printf("Resuming current arm servo positions\n");
      conv[0] = PCA9685_REG_LED2_ON_L;
      conv[1] = 0x00;  // On
      conv[2] = 0x00;
      conv[3] = (unsigned char) ccsrState.armPulseWidth & 0xFF;  
      conv[4] = (unsigned char) (ccsrState.armPulseWidth >> 8 ) & 0x0F;
      conv[5] = 0x00;  // On
      conv[6] = 0x00;
      conv[7] = (unsigned char) ccsrState.elbowPulseWidth & 0xFF;  
      conv[8] = (unsigned char) (ccsrState.elbowPulseWidth >> 8 ) & 0x0F;
      conv[9] = 0x00;  // On
      conv[10] = 0x00;
      conv[11] = (unsigned char) ccsrState.wristPulseWidth & 0xFF;  
      conv[12] = (unsigned char) (ccsrState.wristPulseWidth >> 8 ) & 0x0F;
      conv[13] = 0x00;  // On
      conv[14] = 0x00;
      conv[15] = (unsigned char) ccsrState.handPulseWidth & 0xFF;  
      conv[16] = (unsigned char) (ccsrState.handPulseWidth >> 8 ) & 0x0F;
   }
   else {
      printf("Turning off arm servo's\n");
      conv[0] = PCA9685_REG_LED2_ON_L;
      conv[1] = 0x00;  // On
      conv[2] = 0x00;
      conv[3] = 0x00;
      conv[4] = (unsigned char) (LED_FULL_OFF << LED_FULL_OFF_offset) & 0xFF;
      conv[5] = 0x00;  // On
      conv[6] = 0x00;
      conv[7] = 0x00;
      conv[8] = (unsigned char) (LED_FULL_OFF << LED_FULL_OFF_offset) & 0xFF;
      conv[9] = 0x00;  // On
      conv[10] = 0x00;
      conv[11] = 0x00;
      conv[12] = (unsigned char) (LED_FULL_OFF << LED_FULL_OFF_offset) & 0xFF;
      conv[13] = 0x00;  // On
      conv[14] = 0x00;
      conv[15] = 0x00;
      conv[16] = (unsigned char) (LED_FULL_OFF << LED_FULL_OFF_offset) & 0xFF;
   }
   
      pthread_mutex_lock(&semI2c);
 
      if(ioctl(i2cbus, I2C_SLAVE, PCA9685_ADDR)) {
         logMsg(logFile, "Can't set PCA9685_ADDR I2C address", ERROR);
      }
      usleep(I2C_DELAY);
 
 
      if(write(i2cbus, conv, 17) != 17) {
         logMsg(logFile, "Unsuccessful cmd write to PCA9685", ERROR);
      }
      usleep(I2C_DELAY);
      pthread_mutex_unlock(&semI2c);
} 




// Set arm to specific position by specifying position of arm (shoulder), elbow, wrist and hand. 
// Arms moves at <speed> expressed as percentage of max: speed[0..100]
// Joint values expressed in degrees:
// arm [0..45] (shoulder)
// elbow [0..180]
// wrist [0..180]
// hand  [0..180]
// 'Folded' position is (arm, elbow, wrist, hand) = (45, 0, 0, 0)
void setArm(int arm, int elbow, int wrist, int hand, int speed) {
   unsigned char conv[17];
   int n,m;
   int armPW, elbowPW, wristPW, handPW;
   int armPWInc, elbowPWInc, wristPWInc, handPWInc;
   int delay;
   int armShift, elbowShift, wristShift, handShift;
   
   // shift range -90..90 deg. to 0..180  
   armShift   = arm;
   elbowShift = ELBOW_ANGLE_RANGE - elbow;
   wristShift = wrist;
   handShift  = hand ;
   
   // speed = 0, 1 sec per step
   // speed = 100, 0 sec per step 
   delay = 10000 - speed * 100;

   
   // 100% = SERVOMAX, 0% = SERVOMIN
   armPW = ARM_SERVO_MIN + armShift*(ARM_SERVO_MAX-ARM_SERVO_MIN)/ARM_ANGLE_RANGE;
   elbowPW = ELBOW_SERVO_MIN + elbowShift*(ELBOW_SERVO_MAX-ELBOW_SERVO_MIN)/ELBOW_ANGLE_RANGE;
   wristPW = WRIST_SERVO_MIN + wristShift*(WRIST_SERVO_MAX-WRIST_SERVO_MIN)/WRIST_ANGLE_RANGE;
   handPW = HAND_SERVO_MIN + handShift*(HAND_SERVO_MAX-HAND_SERVO_MIN)/HAND_ANGLE_RANGE;


   if(speed == 100) {
      armPWInc = armPW - ccsrState.armPulseWidth;
      elbowPWInc = elbowPW - ccsrState.elbowPulseWidth;
      wristPWInc = wristPW - ccsrState.wristPulseWidth;
      handPWInc = handPW - ccsrState.handPulseWidth;
   }
   else {
      if(ccsrState.armPulseWidth > armPW) {
         armPWInc = -PW_STEP;
      }
      else {
         armPWInc = PW_STEP;
      }
      if(ccsrState.elbowPulseWidth > elbowPW) {
         elbowPWInc = -PW_STEP;
      }
      else {
         elbowPWInc = PW_STEP;
      }
      if(ccsrState.wristPulseWidth > wristPW) {
         wristPWInc = -PW_STEP;
      }
      else {
         wristPWInc = PW_STEP;
      }
      if(ccsrState.handPulseWidth > handPW) {
         handPWInc = -PW_STEP;
      }
      else {
         handPWInc = PW_STEP;
      }
   }
   while((ccsrState.armPulseWidth!=armPW) ||
         (ccsrState.elbowPulseWidth!=elbowPW) ||
	 (ccsrState.wristPulseWidth!=wristPW) ||
	 (ccsrState.handPulseWidth!=handPW)){
     ccsrState.armPulseWidth = ccsrState.armPulseWidth + armPWInc;
     ccsrState.elbowPulseWidth = ccsrState.elbowPulseWidth + elbowPWInc;
     ccsrState.wristPulseWidth = ccsrState.wristPulseWidth + wristPWInc;
     ccsrState.handPulseWidth = ccsrState.handPulseWidth + handPWInc;
     if(armPWInc>0) {
         if(ccsrState.armPulseWidth>=armPW) {
            ccsrState.armPulseWidth = armPW;
         }
      }
      else {
         if(ccsrState.armPulseWidth<armPW) {
            ccsrState.armPulseWidth = armPW;
         }
      }
     if(elbowPWInc>0) {
         if(ccsrState.elbowPulseWidth>=elbowPW) {
            ccsrState.elbowPulseWidth = elbowPW;
         }
      }
      else {
         if(ccsrState.elbowPulseWidth<elbowPW) {
            ccsrState.elbowPulseWidth = elbowPW;
         }
      }
     if(wristPWInc>0) {
         if(ccsrState.wristPulseWidth>=wristPW) {
            ccsrState.wristPulseWidth = wristPW;
         }
      }
      else {
         if(ccsrState.wristPulseWidth<wristPW) {
            ccsrState.wristPulseWidth = wristPW;
         }
      }
     if(handPWInc>0) {
         if(ccsrState.handPulseWidth>=handPW) {
            ccsrState.handPulseWidth = handPW;
         }
      }
      else {
         if(ccsrState.handPulseWidth<handPW) {
            ccsrState.handPulseWidth = handPW;
         }
      }
      conv[0] = PCA9685_REG_LED2_ON_L;
      conv[1] = 0x00;  // On
      conv[2] = 0x00;
      conv[3] = (unsigned char) ccsrState.armPulseWidth & 0xFF;  
      conv[4] = (unsigned char) (ccsrState.armPulseWidth >> 8 ) & 0x0F;
      conv[5] = 0x00;  // On
      conv[6] = 0x00;
      conv[7] = (unsigned char) ccsrState.elbowPulseWidth & 0xFF;  
      conv[8] = (unsigned char) (ccsrState.elbowPulseWidth >> 8 ) & 0x0F;
      conv[9] = 0x00;  // On
      conv[10] = 0x00;
      conv[11] = (unsigned char) ccsrState.wristPulseWidth & 0xFF;  
      conv[12] = (unsigned char) (ccsrState.wristPulseWidth >> 8 ) & 0x0F;
      conv[13] = 0x00;  // On
      conv[14] = 0x00;
      conv[15] = (unsigned char) ccsrState.handPulseWidth & 0xFF;  
      conv[16] = (unsigned char) (ccsrState.handPulseWidth >> 8 ) & 0x0F;


      pthread_mutex_lock(&semI2c);
 
      if(ioctl(i2cbus, I2C_SLAVE, PCA9685_ADDR)) {
         logMsg(logFile, "Can't set PCA9685_ADDR I2C address", ERROR);
      }
      usleep(I2C_DELAY);
 
 
      if(write(i2cbus, conv, 17) != 17) {
         logMsg(logFile, "Unsuccessful cmd write to PCA9685", ERROR);
      }
      usleep(I2C_DELAY);
      pthread_mutex_unlock(&semI2c);
      usleep(delay);
   }
//   printf("%d %d %d %d\n",      ccsrState.armPulseWidth, ccsrState.elbowPulseWidth , ccsrState.wristPulseWidth, ccsrState.handPulseWidth);


   ccsrState.arm = arm;
   ccsrState.elbow = elbow;
   ccsrState.wrist = wrist;
   ccsrState.hand = hand;
   
} 

// Head pan/tilt test, just look left/right/up/down, no pass/fail check
int pantiltDiagnostics() {
   int speed;

   speed = 70;
   setPanTilt(0, 0, speed);
   setPanTilt(80, 0, speed);
   setPanTilt(-80, 0, speed);
   setPanTilt(0, 0, speed);
   setPanTilt(0, 40, speed);
   setPanTilt(0, -40, speed);
   setPanTilt(0, 0, speed);

   return 1;
} 

/*
void retractArm(){
   setArm(45, 5, 0, 0, 100);
void extendArm(){
   setArm(45, 5, 0, 0, 100);
*/


// Pthread process that keeps camera (the head using pan/tilt) centered on specific tracked object.
// Visual.cpp will set ccsrState.objectTracked to 1 if it finds a target object in camera vision
// If so, and if enabled with ccsrState.trackTargetColorOn (default 0), this process will 
void *camtrack() {


   int pan, tilt;
   int xDelta, yDelta;
   int xDeltaAbs, yDeltaAbs;
   int xDeltaSign, yDeltaSign;
   int xDeltaQ, yDeltaQ;
 
   int tracking;
   tracking = 0;
 
   while(1) {
      usleep(CAMERA_TRACK_INTERVAL);
      if (ccsrState.objectTracked & ccsrState.trackTargetColorOn)
      {
	 // Visual has the target object in view, and tracking is enabled.
	 if(!tracking) {
 	    // We weren't previously tracking, so make sound indicating we are starting to track object.
	    write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[singleA]));
	    lcdEvent = EVENT_TRACKING_OBJECT;
 	    write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
 	 }
	 tracking = 1;

	 // Determine how fas off object is from image center
	 xDelta = (100*(ccsrState.targetVisualObject_X - X_MID_IMAGE))/X_MID_IMAGE;
   	 yDelta = (100*(ccsrState.targetVisualObject_Y - Y_MID_IMAGE))/Y_MID_IMAGE;
   	 xDeltaAbs = abs(xDelta);
   	 yDeltaAbs = abs(yDelta);
         if(xDelta>0) {
	    xDeltaSign = 1;
	 }
	 else {
	    xDeltaSign = -1;
	 }
         if(yDelta>0) {
	    yDeltaSign = 1;
	 }
	 else {
	    yDeltaSign = -1;
	 }

         // Based on distance the camera head has to go, we calculate a step (in degrees) by which we move the 
	 // pan/tilt servos to move object closer to center. The move we still have to go, the bigger the step.
         // We have some hysteresis, to prevent the camera from continuously moving based on noise, this
	 // is currently set to 20 degrees. So if we're less tha 20 deg off, we consider ourselves locked. 
	 // This may be too inaccurate to picking up objects with the arm, so may need to finetune this.
         // For >20 deg offset, we define a simple quantization function:
	 if(xDeltaAbs < 20) {
	    xDeltaQ = 0;
	 }
	 else if ((xDeltaAbs >= 20) && (xDeltaAbs < 40)) {
	    xDeltaQ = 1 * xDeltaSign;
	 }
	 else if ((xDeltaAbs >= 40) && (xDeltaAbs < 70)) {
	    xDeltaQ = 2 * xDeltaSign;
	 }
	 else {
	    xDeltaQ = 5 * xDeltaSign;
	 } 

         if(yDeltaAbs < 20) {
	    yDeltaQ = 0;
	 }
	 else if ((yDeltaAbs >= 20) && (yDeltaAbs < 40)) {
	    yDeltaQ = 1 * yDeltaSign;
	 }
	 else if ((yDeltaAbs >= 40) && (yDeltaAbs < 70)) {
	    yDeltaQ = 2 * yDeltaSign;
	 }
	 else {
	    yDeltaQ = 4 * yDeltaSign;
	 } 

         if(xDeltaQ == 0){  // only pan for now
            // We consider ourselves locked on target if camera pan is less than 20 deg off.
	    // We don't consider tilt value (yet), because only pan is important to turn CCSR in the right direction
	    if(ccsrState.trackedObjectCentered == 0) {
   	       //If we weren't previously centered before, sound happy alarm that now we do.
//	       write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[singleA]));
	       lcdEvent = EVENT_TARGET_LOCKED;
 	       write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
            }
	    ccsrState.trackedObjectCentered = 1;
	    // Key activity here:
            // If we are centered on target, we want to drive toward this target. THerefor continuously set
            // target heading to where the camera is pointed. The process *drivetoTarget in motors.c will continuously
            // adjust heading to target heading if enabled. Note this means that if we manually want to set a target
            // heading, trackTargetColorOn must be off.
            // Open: targetHeading only gets updated if cam is centered, what if we drive too fast and cam can;t keep up: target heading will never
            // be updated? Should driveToTarget also check pan?
            ccsrState.targetHeading = addAngleToHeading(ccsrState.pan);
	 }
	 else {
	    ccsrState.trackedObjectCentered = 0;
         }
	 
 //        printf("%d %d %d %d\n",  xDelta , xDeltaQ, yDelta , yDeltaQ);       

         // Calculate new pan/tilr positions for head:
 	 pan = ccsrState.pan + xDeltaQ;
 	 if(pan>80) {
 	    pan = 80;
 	 }
 	 if(pan<-80) {
 	    pan = -80;
 	 }
 	 tilt = ccsrState.tilt - yDeltaQ;
 	 if(tilt<-40) {
 	    tilt = -40;
 	 }
 	 if(tilt>40) {
 	    tilt = 40;
 	 }
 
   	 // Set head, use 70% of speed.
	 setPanTilt(pan, tilt, 70);
      }
      else {
         tracking = 0;
      }	 
   } 
}
