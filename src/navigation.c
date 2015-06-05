#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <math.h>
#include "ccsr.h"
#include "navigation.h"
#include "utils.h"
#include <linux/i2c-dev.h>


#ifndef I2C_SLAVE
#define I2C_SLAVE 0
#endif

extern FILE *logFile;
extern int i2cbus;
extern pthread_mutex_t semI2c;
extern ccsrStateType ccsrState;
extern devRandom;     
int dfd;
char string[100];
char navdmpfile[] = NAV_DUMP;



void configNav() {
   unsigned char buffer[4];
   unsigned char conv[2];
   int  config_reg = 0;
   int  result;

   pthread_mutex_lock(&semI2c);

   // Configure Compass
   if(ioctl(i2cbus, I2C_SLAVE, COMPASS_ADDR)) {
      logMsg(logFile, "Can't set COMPASS_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   config_reg = (TEMP_EN << TEMP_EN_offset) |
	        (DO_15HZ << DO_offset);
   buffer[0] = (char) CRA_REG_M;
   buffer[1] = (char) config_reg  & 0xFF;
   if(write(i2cbus, buffer, 2) != 2) {
      logMsg(logFile, "Unsuccessful cmd write to COMPASS", ERROR);	  
   }
   usleep(I2C_DELAY);
   config_reg = (MD_CONT << MD_offset);
   buffer[0] = (char) MR_REG_M;
   buffer[1] = (char) config_reg  & 0xFF;
   if(write(i2cbus, buffer, 2) != 2) {
      logMsg(logFile, "Unsuccessful cmd write to COMPASS", ERROR);	  
   }
   usleep(I2C_DELAY);

   // Configure Linear Accellerator
   if(ioctl(i2cbus, I2C_SLAVE, LIN_ACC_ADDR)) {
      logMsg(logFile, "Can't set LIN_ACC_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   config_reg = (ODR_NORMAL_1HZ << ODR_offset) |
	        (Z1Y1X1 << ZYX_EN_offset);
   buffer[0] = (char) CTRL_REG1_A;
   buffer[1] = (char) config_reg  & 0xFF;
   if(write(i2cbus, buffer, 2) != 2) {
      logMsg(logFile, "Unsuccessful cmd write to COMPASS", ERROR);	  
   }
   usleep(I2C_DELAY);

   // Configure Gyro
   if(ioctl(i2cbus, I2C_SLAVE, GYRO_ADDR)) {
      logMsg(logFile, "Can't set GYRO_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   config_reg = (ODR_NORMAL_95HZ125CO << ODR_offset) |
                (G_PD_NORMAL << G_PD_offset) |
	        (Z1Y1X1 << ZYX_EN_offset);
   buffer[0] = (char) CTRL_REG1_G;
   buffer[1] = (char) config_reg  & 0xFF;
   if(write(i2cbus, buffer, 2) != 2) {
      logMsg(logFile, "Unsuccessful cmd write to GYRO", ERROR);	  
   }
   usleep(I2C_DELAY);

   pthread_mutex_unlock(&semI2c);
} 


int getTemp() {
   int result;
   char conv[2];
   char* conv_h = &conv[0];
   char* conv_l = &conv[1];
   char buffer[4];

#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   // no auto-increment for address > 12?
   if(ioctl(i2cbus, I2C_SLAVE, COMPASS_ADDR)) {
      logMsg(logFile, "Can't set COMPASS_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   buffer[0] = (char) TEMP_OUT_H_M;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful TEMP_OUT_H_M write to COMPASS", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv_h, 1) != 1) {
      logMsg(logFile, "Unsuccessful TEMP_OUT_H/L_M read from COMPASS", ERROR);	   
   }
   buffer[0] = (char) TEMP_OUT_L_M ;
   usleep(I2C_DELAY);
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful TEMP_OUT_L_M write to COMPASS", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv_l, 1) != 1) {
      logMsg(logFile, "Unsuccessful TEMP_OUT_H/L_M read from COMPASS", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
   result = (int) (( conv[0] << 4 ) | ((conv[1] >> 4) & 0xF)) & 0xFFF;
#endif   
 //  printf("temp %d\n", result);
   return result;
} 

int getHeading() {
   int result_x, result_y, result_z;
   double ang;
   double absx, absy;
   unsigned char conv[6];
   char buffer[4];

#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, COMPASS_ADDR)) {
      logMsg(logFile, "Can't set COMPASS_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   buffer[0] = (char) OUT_X_H_M | AUTO_INC;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful OUT_X/Y/Z_H/L_M write to COMPASS", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 6) != 6) {
      logMsg(logFile, "Unsuccessful OUT_X/Y/Z_H/L_M  read from COMPASS", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
   result_x = (short) (conv[0] << 8 | conv[1]);
   result_z = (short) (conv[2] << 8 | conv[3]);
   result_y = (short) (conv[4] << 8 | conv[5]);

   ccsrState.compassRawFieldX = result_x;
   ccsrState.compassRawFieldY = result_y;
//   printf("compassX %d compassY %d\n", result_x, result_y);

   // Compass compensation, based on calibration data
   result_x = -(result_x - ccsrState.compassCalibrationOffsetX);
   result_y =   result_y - ccsrState.compassCalibrationOffsetY;

   //    printf("compassX %d compassZ %d compassY %d\n", result_x, result_z, result_y);


   //printf("compassX %d %d %d \n", result_x, conv[0], conv[1]);

   absx = (double) abs(result_x);
   absy = (double) abs(result_y);
   ang = atan(absx/absy);
   ang = 360*ang/6.28;
   
   if((result_x > 0) && (result_y < 0)) {
      ang = 180-ang;
   }
   else if((result_x < 0) && (result_y < 0)) {
      ang = 180+ang;
   }
   else if((result_x < 0) && (result_y > 0)) {
      ang = 360-ang;
   }
   //printf("compassX %d compassY %d ang %f\n", result_x, result_y, ang);

#endif   
   return (int) ang;
} 


int calibrateCompass() {

   int nav_prev;
   int maxXField;
   int minXField;
   int maxYField;
   int minYField;
   int heading_prev;
   int n;
   int midX;
   int midY;
   
   nav_prev = ccsrState.navigationOn;
   ccsrState.navigationOn = 1;

   say("Calibrating compass");
   // Initiate slow turn
   while(!speedFiltered(0, 210)){
      brainCycle();
   }
   
   maxXField = -100000;
   minXField = 100000;
   maxYField = -100000;
   minYField = 100000;
   
   heading_prev = ccsrState.heading;
   
   for(n=0;n<300;n++) {   //10 sec?
      if(ccsrState.compassRawFieldX > maxXField) {
         maxXField = ccsrState.compassRawFieldX;
      }
      if(ccsrState.compassRawFieldX < minXField) {
         minXField = ccsrState.compassRawFieldX;
      }
      if(ccsrState.compassRawFieldY > maxYField) {
         maxYField = ccsrState.compassRawFieldY;
      }
      if(ccsrState.compassRawFieldY < minYField) {
         minYField = ccsrState.compassRawFieldY;
      }
      brainCycle();
  }

   // Stop turn
   while(!speedFiltered(0, 0)) {
      brainCycle();
   }

   midX = (maxXField - minXField)/2;
   ccsrState.compassCalibrationOffsetX = maxXField - midX;
   midY = (maxYField - minYField)/2;
   ccsrState.compassCalibrationOffsetY = maxYField - midY;

   printf("maxfields Xmax:%d Xmin:%d Ymax:%d Ymin:%d\n", maxXField, minXField, maxYField, minYField);

   printf("Compas Calibration X:%d Y:%d\n", ccsrState.compassCalibrationOffsetX, ccsrState.compassCalibrationOffsetY);
   ccsrState.navigationOn = nav_prev;



}


int getLinAcc() {
   int result_x, result_y, result_z;
   unsigned char conv[6];
   char buffer[4];

#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, LIN_ACC_ADDR)) {
      logMsg(logFile, "Can't set LIN_ACC_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   buffer[0] = (char) OUT_X_L_A | AUTO_INC;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful OUT_X/Y/Z_H/L_M write to LinAcc", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 6) != 6) {
      logMsg(logFile, "Unsuccessful OUT_X/Y/Z_H/L_M  read from linAcc", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
   result_x = (short) (conv[1] << 8 | conv[0]);
   result_z = (short) (conv[5] << 8 | conv[4]);
   result_y = (short) (conv[3] << 8 | conv[2]);

  // printf("linAccX %d linAccZ %d linAccY %d\n", result_x, result_z, result_y);
  // printf("linAccX %d %d %d\n", result_x,  conv[1],  conv[0]);
  // printf("%d %d\n", (unsigned char) conv[0], (unsigned char) conv[1]);

//     sprintf(dfd,"%d, %d, %d, %d\n", ccsrState.timer, result_x, result_z, result_y);

#endif   
//   return result_x;
} 

int getGyro() {
   int result_x, result_y, result_z;
   unsigned char conv[6];
   char buffer[4];

#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, GYRO_ADDR)) {
      logMsg(logFile, "Can't set GYRO_ADDR I2C address", ERROR);
   }
   usleep(I2C_DELAY);
   buffer[0] = (char) OUT_X_L_G | AUTO_INC;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful OUT_X/Y/Z_H/L_M write to GYRO_ADDR", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 6) != 6) {
      logMsg(logFile, "Unsuccessful OUT_X/Y/Z_H/L_M  read from GYRO_ADDR", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);
   result_x = (short) (conv[1] << 8 | conv[0]);
   result_y = (short) (conv[5] << 8 | conv[4]);
   result_z = (short) (conv[3] << 8 | conv[2]);

   ccsrState.gyroAngMoment_X = result_x;
   ccsrState.gyroAngMoment_Y = result_y;
   ccsrState.gyroAngMoment_Z = result_z;

   printf("linAccX %d linAccZ %d linAccY %d\n", result_x, result_z, result_y);
//   printf("GyroX %d %d %d\n", result_x,  conv[1],  conv[0]);
//   printf("GyroX %d %d %d %d %d %d \n", conv[5], conv[4], conv[3], conv[2],  conv[1],  conv[0]);
  // printf("%d %d\n", (unsigned char) conv[0], (unsigned char) conv[1]);



#endif   
//   return result_x;
} 


void *navigation() {

   logMsg(logFile, "Starting navigation", LOG); 
   configNav();

   dfd = open(navdmpfile, O_WRONLY);
   sprintf(string,"time, linaccX, linaccZ, linaccY\n");
   write(dfd, string, strlen(string));
// need to close file!

   while(1) {
      usleep(NAV_SENSE_PERIOD);
      if(ccsrState.navigationOn) {
         ccsrState.heading = getHeading();
      }
      if(ccsrState.gyroOn) {
         getGyro();
      }
//         getLinAcc();
//         getGyro();
   }
}

