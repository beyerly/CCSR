#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include "ccsr.h"
#include "irSensors.h"
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


void configVNCL4000() {
   char buffer[4];
   char  config_reg;

   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, VNCL4000_ADDR)) {
      logMsg(logFile, "Can't set VNCL4000 I2C address", ERROR);
   }
   usleep(I2C_DELAY);

   config_reg = (ALS_OD_START << ALS_OD_offset) |
 		(PROX_OD_START << PROX_OD_offset);
 


   buffer[0] = (char) REG0_CMD;
   buffer[1] = (char) config_reg  & 0xFF;

   if(write(i2cbus, buffer, 2) != 2) {
      logMsg(logFile, "Unsuccessful cmd write to ADC0", ERROR);	  
   }
   usleep(I2C_DELAY);

  config_reg = 2;

  buffer[0] = (char) IR_LED_CUR_SET;
  buffer[1] = (char) config_reg & 0xFF;

  if(write(i2cbus, buffer, 2) != 2) {
	exit(0);
  }	

   usleep(I2C_DELAY);

   pthread_mutex_unlock(&semI2c);

} 



void configADC(int sensor) {
   char buffer[4];
   char conv[2];
   int  config_reg;
   int  result;

   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, ADC0_ADDR)) {
      logMsg(logFile, "Can't set ADC0 I2C address", ERROR);
   }
   usleep(I2C_DELAY);

   config_reg = (OS_0 << OS_offset) |
 		(sensor << MUX_offset) |
 		(PGA_1x << PGA_offset) |
 		(MODE_CONT << MODE_offset) |
 		(DR_128 << DR_offset) |
 		(COMP_MODE_TRAD << COMP_MODE_offset) |
 		(COMP_POLL_AL << COMP_POLL_offset) |
 		(COMP_Q_disable << COMP_Q_offset);
 

   buffer[0] = (char) ADC_REG_CNFG;
   buffer[1] = (char) (config_reg >> 8) & 0xFF;
   buffer[2] = (char) config_reg  & 0xFF;

   if(write(i2cbus, buffer, 3) != 3) {
      logMsg(logFile, "Unsuccessful cmd write to ADC0", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

} 




int readADC(int sensor) {
   char buffer[4];
   char conv[2];
   int config_reg;
   int result;
   int AD_sleep = 0;
      
#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, ADC0_ADDR)) {
      logMsg(logFile, "Can't set ADC0 I2C address", ERROR);
   }
   usleep(I2C_DELAY);


   buffer[0] = (char) ADC_REG_CONV;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful ADC_REG_CONV write to ADC0", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 2) != 2) {
      logMsg(logFile, "Unsuccessful ADC_REG_CONV read from ADC0", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

   result = (int) (( conv[0] << 4 ) | ((conv[1] >> 4) & 0xF)) & 0xFFF;
#else
   if(read(devRandom, conv, 1) != 1) {
      logMsg(logFile, "Unsuccessful read from /dev/random", ERROR);	   
   }
   result = (MAX_IR_RANGE * abs(conv[0]))/128;
#endif   
   return result;
} 


int getIrDistance(int sensor) {
   int result;
      
   result = readADC(sensor);
   if(result<MIN_IR_RANGE) {
      logMsg(logFile, "Clipping out of range ADC0.ADC_REG_CONV", ERROR);	  
      result = MIN_IR_RANGE;
   }
   else if (result>MAX_IR_RANGE) {
      logMsg(logFile, "Clipping out of range ADC0.ADC_REG_CONV", ERROR);	  
      result = MAX_IR_RANGE;
   }
   return result;
} 

int getSonarDistance(int sensor) {
   int result;
      
   result = readADC(sensor);
   if(result<MIN_SONAR_RANGE) {
      logMsg(logFile, "Clipping out of range ADC0.ADC_REG_CONV", ERROR);	  
      result = MIN_IR_RANGE;
   }
   else if (result>MAX_SONAR_RANGE) {
      logMsg(logFile, "Clipping out of range ADC0.ADC_REG_CONV", ERROR);	  
      result = MAX_SONAR_RANGE;
   }
   return result;
} 

int getAmbientLight() {
   char buffer[4];
   unsigned char conv[2];
   int config_reg;
   int result;
      
#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, VNCL4000_ADDR)) {
      logMsg(logFile, "Can't set VNCL4000 I2C address", ERROR);
   }
   usleep(I2C_DELAY);

   buffer[0] = (char) AMB_LIGHT_RES_H;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful ADC_REG_CONV write to ADC0", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 2) != 2) {
      logMsg(logFile, "Unsuccessful ADC_REG_CONV read from ADC0", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

   result = (int) (( conv[0] << 8 ) | conv[1]);
//   printf("ambient %d %d %d\n", conv[0], conv[1], result);
#endif   
   return result;
} 

int getProximity() {
   char buffer[4];
   unsigned char conv[2];
   int config_reg;
   int result;
      
#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, VNCL4000_ADDR)) {
      logMsg(logFile, "Can't set VNCL4000 I2C address", ERROR);
   }
   usleep(I2C_DELAY);

   buffer[0] = (char) PROX_RES_H;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful ADC_REG_CONV write to ADC0", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 2) != 2) {
      logMsg(logFile, "Unsuccessful ADC_REG_CONV read from ADC0", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

   result = (int) (( conv[0] << 8 ) | conv[1]);
//   printf("proximity %d %d %d\n", conv[0], conv[1], result);
#endif   
   return result;
} 


void *proximitySensors() {

   logMsg(logFile, "Starting IR distance sensors", LOG); 

   while(1) {
      if(ccsrState.proximitySensorsOn) {
 	 configADC(MUX_AIN0);
         usleep(IR_SENSE_PERIOD);
 	 ccsrState.irDistFrontLeft = getIrDistance(MUX_AIN0);

 	 configADC(MUX_AIN1);
         usleep(IR_SENSE_PERIOD);
 	 ccsrState.irDistFrontRight = getIrDistance(MUX_AIN1);

// 	 configADC(MUX_AIN3);
//         usleep(IR_SENSE_PERIOD);
// 	 ccsrState.irDistBelow = getSonarDistance(MUX_AIN3);



//         ccsrState.proximity = getProximity();
//          printf("irLeft %d irRight %d below %d\n", ccsrState.irDistFrontLeft, ccsrState.irDistFrontRight, ccsrState.irDistBelow);
      }
      if(ccsrState.sonarSensorsOn) {
 	 configADC(MUX_AIN2);
         usleep(IR_SENSE_PERIOD);
 	 ccsrState.sonarDistFront = getSonarDistance(MUX_AIN2);
         //printf("sonar %d\n", ccsrState.sonarDistFront);
      }
   }
}


void *sonarSensors() {

   logMsg(logFile, "Starting Sonar", LOG); 

   while(1) {
      if(ccsrState.sonarSensorsOn) {
 	 configADC(MUX_AIN2);
 	 ccsrState.sonarDistFront = getSonarDistance(MUX_AIN2);
      }
      usleep(SONAR_SENSE_PERIOD);

     
     printf("sonar %d\n", ccsrState.sonarDistFront);

   }
}

void *environmentalSensors() {

   logMsg(logFile, "Starting environmental sensors", LOG); 

   while(1) {

      if(ccsrState.environmantalSensorsOn) {
         configVNCL4000();
         ccsrState.ambientLight = getAmbientLight();
         ccsrState.temp = getTemp();
      }
      usleep(ENV_SENSE_PERIOD);
   }
}
