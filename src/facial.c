

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


char* dispBuf;

/*char eye_open_bmp[] =
  { 0x00,
    0x80,
    0xFF,
    0x81,
    0xFF,
    0x83,
    0xFF,
    0x87,
    0xFF,

    0x0F,
    0xFF,
    0x1F,
    0xFF,
    0x3F,
    0xFF,
    0x7F,
    0x00,
    0x00,
    0x00
    };
*/    
    
 char eye_open_bmp[] =
  { 
    0x00,
    0x18,
    0x3C,
    0x7E,
    0x7E,
    0x3C,
    0x18,
    0x00
    };
   
    
    
char eye_closed_bmp[] =
  { 
  0x00,
  0x0,
    0x0,
    0x0,
    0x0,
    0x0,
    0x0,
    0x0};

void facialInit() {
   char buffer[4];

   buffer[0] = 0x21;  // turn on oscillator
   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, EYE_R_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
//   if(write(i2cbus, buffer,1 ) !=1 ) {
//      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
//   }
   printf("ic snd %d|n", write(i2cbus, buffer,1 ));
/*   usleep(I2C_DELAY);
   if(ioctl(i2cbus, I2C_SLAVE, EYE_L_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
*/   pthread_mutex_unlock(&semI2c);
printf("init facial\n");

  eyesSetBlinkRate(HT16K33_BLINK_OFF);
  eyesSetBrightness(1); // max brightness
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

/*   if(ioctl(i2cbus, I2C_SLAVE, EYE_L_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
*/
   pthread_mutex_unlock(&semI2c);
printf("bright facial\n");

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

/*   if(ioctl(i2cbus, I2C_SLAVE, EYE_L_ADDR)) {
      logMsg(logFile, "Can't set LCD_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer,1 ) !=1 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
*/
   pthread_mutex_unlock(&semI2c);
printf("blink facial\n");
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

   if(write(i2cbus, dispBuff, 16 ) != 16 ) {
      logMsg(logFile, "Unsuccessful cmd write to LCD_ADDR", ERROR);	  
   }
//      printf("ic wr %d\n", write(i2cbus, dispBuff,8 ));
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

}




void draw(char* dispBuf, char* bitmap) {

int x;
int y;

y=0;
dispBuf[0]=0x00;

for(x=1;x<=16;x=x+2){

dispBuf[x] = (char) (((bitmap[y] << 7) & 0x80) | ((bitmap[y] >> 1) & 0x7F)) & 0xFF;
printf("%x\n",dispBuf[x]);
y=y+1;

}

//exit(0);
}


void *facialExpressions() {
   expressionType expr;
   int result;


int i,q;


dispBuf = (char*) malloc(17 *sizeof(char));

   while(1) {
      result = read (pipeFacialMsg[OUT],&expr,sizeof(expr));
      printf("received %d\n", expr.type);
      switch(expr.type) {
 	 case EXPR_BLINK:
for (i=0;i<10;i++){
	    draw(dispBuf, eye_closed_bmp);
for (q=0;q<17;q++){
printf("%x\n",dispBuf[q]);
}
	    eyesWriteDisplay(EYE_R_ADDR, dispBuf);
            usleep(500000);
	    draw(dispBuf, eye_open_bmp);
	    eyesWriteDisplay(EYE_R_ADDR, dispBuf);
            usleep(500000);
}
         break;
 	 case EXPR_TALK:
         break;
      }
   }
}
