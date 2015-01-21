

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <linux/i2c-dev.h>
#include "ccsr.h"
#include "facial.h"
#include "utils.h"

#ifndef I2C_SLAVE
#define I2C_SLAVE 0
#endif

extern FILE *logFile;
extern ccsrStateType ccsrState;
extern int i2cbus;
extern pthread_mutex_t semI2c;
extern int pipeFacialMsg[2];
extern int devRandom;

char* dispBuf;
char* dispBufBase;
   char* eyeL_bmp;
   char* eyeR_bmp;


char eye_open_bmp[] =
   { 
   0x3C,
   0x7E,
   0xE7,
   0x3C,
   0x3C,
   0xE7,
   0x7E,
   0x3C
   };

char eye_left_bmp[] =
   { 
   0x3C,
   0x7E,
   0xCF,
   0x87,
   0x87,
   0xCF,
   0x7E,
   0x3C
   };

char eye_right_bmp[] =
   { 
   0x3C,
   0x7E,
   0xF3,
   0xE1,
   0xE1,
   0xF3,
   0x7E,
   0x3C
   };

char eye_up_bmp[] =
   { 
   0x3C,
   0xE7,
   0xC3,
   0xC3,
   0xE7,
   0xFF,
   0x7E,
   0x3C
   };

char eye_down_bmp[] =
   { 
   0x3C,
   0x7E,
   0xFF,
   0xE7,
   0xC3,
   0xC3,
   0xE7,
   0x3C
   };

char eye_sleep_bmp[] =
   { 
   0x00,
   0x00,
   0x00,
   0x7E,
   0x81,
   0x00,
   0x00,
   0x00
   };

// Angry as left eye, scared as right eye
char eye_posangle_bmp[] =
   { 
   0x00,
   0x60,
   0xD8,
   0xC6,
   0xC3,
   0xE7,
   0x7E,
   0x3C
   };

// Angry as right eye, scared as left eye
char eye_negangle_bmp[] =
   { 
   0x00,
   0x06,
   0x1B,
   0x63,
   0xC3,
   0xE7,
   0x7E,
   0x3C
   };



char mask_1[] =
   { 
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF
   };

char mask_blink0[] =
   { 
   0x00,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0x00
   };
   
char mask_blink1[] =
   { 
   0x00,
   0x00,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0x00,
   0x00
   };
 
char mask_blink2[] =
   { 
   0x00,
   0x00,
   0x00,
   0xFF,
   0xFF,
   0x00,
   0x00,
   0x00
   };
 
char mask_0[] =
   { 
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00
   };


char mouth_scanner_bmp[] =
   { 
   0x00,
   0x00,
   0x00,
   0x00,
   0x00
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x00,
   0x10,
   0x10,
   0x10
   };

char mouth_open_bmp[] =
   { 
   0x18,
   0x3C,
   0x7E,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0xFF,
   0x7E,
   0x3C,
   0x18
   };

void facialInit() {
   dispEnable(EYE_R_ADDR);
   dispSetBlinkRate(EYE_R_ADDR, HT16K33_BLINK_OFF);
   dispSetBrightness(EYE_R_ADDR, 15); // max brightness
   dispEnable(EYE_L_ADDR);
   dispSetBlinkRate(EYE_L_ADDR, HT16K33_BLINK_OFF);
   dispSetBrightness(EYE_L_ADDR, 15); // max brightness
   dispEnable(MOUTH_ADDR);
   dispSetBlinkRate(MOUTH_ADDR, HT16K33_BLINK_OFF);
   dispSetBrightness(MOUTH_ADDR, 15); // max brightness
}

void dispEnable(int i2cAddr) {
   char buffer[1];

   buffer[0] = 0x21;  // turn on oscillator
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, i2cAddr)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

}

void dispSetBrightness(int i2cAddr, char b) {
   char buffer[4];
   if (b > 15) b = 15;

   buffer[0] = HT16K33_CMD_BRIGHTNESS | b;
   pthread_mutex_lock(&semI2c);
  
   if(ioctl(i2cbus, I2C_SLAVE, i2cAddr)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
}

void dispSetBlinkRate(int i2cAddr, int b) {
   char buffer[4];
   
   if (b > 3) b = 0; // turn off if not sure
   buffer[0] = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1);
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, i2cAddr)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
}


void writeDisplay(int i2cAddr, char* dispBuff) {
   int i;
   char* buffer;
   buffer = dispBuff-1;
   buffer[0] = 0x00;

   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, i2cAddr)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer, DISP_ROWS+1 ) != DISP_ROWS+1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

}



void draw_bmp_8x8(char* dispBuf, char* bitmap, char* mask) {

   int x;
   int y;

   y=0;
   for(x=0;x<DISP_ROWS;x=x+2){
      dispBuf[x] = (char) (((bitmap[y] << 7) & 0x80) | ((bitmap[y] >> 1) & 0x7F)) & mask[y];
      y=y+1;
   }
}

void draw_bmp_16x8(char* dispBuf, char* bitmap, char* mask) {

   int x;
   int y;

   y=0;
   for(x=0;x<DISP_ROWS;x=x+2){
      dispBuf[x] = (char) (((bitmap[y] << 7) & 0x80) | ((bitmap[y] >> 1) & 0x7F)) & mask[y];
      y=y+1;
   }
   for(x=1;x<DISP_ROWS;x=x+2){
      dispBuf[x] = (char) (((bitmap[y] << 7) & 0x80) | ((bitmap[y] >> 1) & 0x7F)) & mask[y];
      y=y+1;
   }
}

void drawEyes(char* maskR, char* maskL){
   draw_bmp_8x8(dispBuf, eyeR_bmp, maskR);
   writeDisplay(EYE_R_ADDR, dispBuf);
   draw_bmp_8x8(dispBuf, eyeR_bmp, maskL);
   writeDisplay(EYE_L_ADDR, dispBuf);
}


void *facialExpressions() {
   expressionType expr;
   int result;
   int i,q;
   char random;

   dispBufBase = (char*) malloc(DISP_ROWS*DISP_PAGES+1 *sizeof(char));
   dispBuf = dispBufBase + 1; // Start on page 0, but leave room for i2c address.

   eyeR_bmp = eye_open_bmp;
   eyeL_bmp = eye_open_bmp;
   drawEyes(mask_1, mask_1);

   while(1) {
      result = read (pipeFacialMsg[OUT],&expr,sizeof(expr));
      printf("facial %d\n", expre.type);
      switch(expr.type) {
 	 case EXPR_BLINK:
            drawEyes(mask_blink0, mask_blink0);
	    usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_0, mask_0);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink0, mask_blink0);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_LOOKSTRAIGHT:
	    eyeR_bmp = eye_open_bmp;
	    eyeL_bmp = eye_open_bmp;
	    drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_LOOKLEFT:
	    eyeR_bmp = eye_left_bmp;
	    eyeL_bmp = eye_left_bmp;
	    drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_LOOKRIGHT:
	    eyeR_bmp = eye_right_bmp;
	    eyeL_bmp = eye_right_bmp;
	    drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_LOOKUP:
	    eyeR_bmp = eye_up_bmp;
	    eyeL_bmp = eye_up_bmp;
	    drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_LOOKDOWN:
	    eyeR_bmp = eye_down_bmp;
	    eyeL_bmp = eye_down_bmp;
	    drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_ANGRY:
            drawEyes(mask_blink0, mask_blink0);
	    usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_0, mask_0);
            usleep(EXPR_BLINK_FRAME_RATE);
	    eyeR_bmp = eye_posangle_bmp;
	    eyeL_bmp = eye_negangle_bmp;
	    drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink0, mask_blink0);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_SCARED:
            drawEyes(mask_blink0, mask_blink0);
	    usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_0, mask_0);
            usleep(EXPR_BLINK_FRAME_RATE);
	    eyeR_bmp = eye_negangle_bmp;
	    eyeL_bmp = eye_posangle_bmp;
	    drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink0, mask_blink0);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_CROSSEYED:
            drawEyes(mask_blink0, mask_blink0);
	    usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_0, mask_0);
            usleep(EXPR_BLINK_FRAME_RATE);
	    eyeR_bmp = eye_left_bmp;
	    eyeL_bmp = eye_right_bmp;
	    drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink0, mask_blink0);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_SLEEP:
            drawEyes(mask_blink0, mask_blink0);
	    usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
	    eyeR_bmp = eye_sleep_bmp;
	    eyeL_bmp = eye_sleep_bmp;
	    drawEyes(mask_blink2, mask_blink2);
         break;
 	 case EXPR_WAKE:
	    eyeR_bmp = eye_open_bmp;
	    eyeL_bmp = eye_open_bmp;
	    drawEyes(mask_blink2, mask_blink2);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink1, mask_blink1);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_blink0, mask_blink0);
            usleep(EXPR_BLINK_FRAME_RATE);
            drawEyes(mask_1, mask_1);
         break;
 	 case EXPR_SCANNER:
	    mouth_bmp = mouth_scanner_bmp;
	    draw_bmp_16x8(dispBuf, mouth_bmp, mask_1_16x8);
            writeDisplay(MOUTH_ADDR, dispBuf);
            usleep(EXPR_BLINK_FRAME_RATE);
            for(i=1;i<13;i++){
	       writeDisplay(MOUTH_ADDR, dispBuf+i);
	       usleep(EXPR_BLINK_FRAME_RATE);
	    }
            for(i=13;i>=1;i--){
	       writeDisplay(MOUTH_ADDR, dispBuf+i);
	       usleep(EXPR_BLINK_FRAME_RATE);
	    }
         break;
 	 case EXPR_TALK:
	    mouth_bmp = mouth_open_bmp;
	    while(i<expr.length){
	       read(devRandom, &random, 1)
	       talkCOunt = (10 * abs(random)/128);
	       draw_bmp_16x8(dispBuf, mouth_bmp, mask_1_16x8);
               writeDisplay(MOUTH_ADDR, dispBuf);
	       for(j=0;j<talkCount;j++){
	          i=i+1;
		  usleep(EXPR_BLINK_FRAME_RATE);
	       }
	       read(devRandom, &random, 1);
	       talkCOunt = (10 * abs(random)/128);
	       draw_bmp_16x8(dispBuf, mouth_bmp, mask_0_16x8);
               writeDisplay(MOUTH_ADDR, dispBuf);
	       for(j=0;j<talkCount;j++){
	          i=i+1;
		  usleep(EXPR_BLINK_FRAME_RATE);
	       }
	    }
         break;
      }
   }
}
