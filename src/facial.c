

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


static const char eye_open_bmp[] =
  { 0xFF,
    0xF0,
    0x0F,
    0x00,
    0xFF,
    0xF0,
    0x0F,
    0x00 };
static const char eye_closed_bmp[] =
  { 0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF };

void facialInit() {
   char buffer[4];

   buffer[0] = 0x21;  // turn on oscillator
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, EYE_R_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(ioctl(i2cbus, I2C_SLAVE, EYE_L_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

  eyesSetBlinkRate(HT16K33_BLINK_OFF);
  eyesSetBrightness(15); // max brightness
}


void eyesSetBrightness(char b) {
   char buffer[4];
   if (b > 15) b = 15;

   buffer[0] = HT16K33_CMD_BRIGHTNESS | b;
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, EYE_R_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);

   if(ioctl(i2cbus, I2C_SLAVE, EYE_L_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);

   pthread_mutex_unlock(&semI2c);

}

void eyesSetBlinkRate(int b) {
   char buffer[4];
   
   if (b > 3) b = 0; // turn off if not sure

   buffer[0] = HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1);
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, EYE_R_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);

   if(ioctl(i2cbus, I2C_SLAVE, EYE_L_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);

   pthread_mutex_unlock(&semI2c);
}



void eyesWriteDisplay(int eye, char* dispBuff) {
   char buffer[4];
   int i;

   buffer[0] = 0x00;
   pthread_mutex_lock(&semI2c);
   printf("writing eye %d\n", eye);
   if(ioctl(i2cbus, I2C_SLAVE, eye)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);

   if(write(i2cbus, dispBuff, 8 ) != 8 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

}



void *facialExpressions() {
   expressionType expr;

   while(1) {
      result = read (pipeFacialMsg[OUT],&expr,sizeof(expr));
      switch(expr.type) {
 	 case EXPR_BLINK:
	    eyesWriteDisplay(EYE_R_ADDR, eye_closed_bmp);
            usleep(1000);
	    eyesWriteDisplay(EYE_R_ADDR, eye_open_bmp);
         break;
 	 case EXPR_TALK:
         break;
      }
   }
}