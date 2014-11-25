#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include "ccsr.h"
#include "powerMonitor.h"
#include "utils.h"
#include <linux/i2c-dev.h>
#include "lcdDisp.h"
#include "sound.h"

#ifndef I2C_SLAVE
#define I2C_SLAVE 0
#endif

extern FILE *logFile;
extern int i2cbus;
extern pthread_mutex_t semI2c;
extern ccsrStateType ccsrState;
extern devRandom;     
extern soundType sound[NUM_SOUNDS];
extern int pipeSoundGen[2];
extern int pipeLCDMsg[2];
char lcdEvent;

void configPowerMonitor() {
   char buffer[4];
   int  config_reg;

   pthread_mutex_lock(&semI2c);

   if(ioctl(i2cbus, I2C_SLAVE, INA219_ADDR)) {
      logMsg(logFile, "Can't set ADC0 I2C address", ERROR);
   }
   usleep(I2C_DELAY);

/*   config_reg = (OS_0 << OS_offset) |
 		(sensor << MUX_offset) |
 		(PGA_1x << PGA_offset) |
 		(MODE_CONT << MODE_offset) |
 		(DR_128 << DR_offset) |
 		(COMP_MODE_TRAD << COMP_MODE_offset) |
 		(COMP_POLL_AL << COMP_POLL_offset) |
 		(COMP_Q_disable << COMP_Q_offset);
*/ 

   buffer[0] = (char) INA219_REG_CNFG;
   buffer[1] = (char) (config_reg >> 8) & 0xFF;
   buffer[2] = (char) config_reg  & 0xFF;

   if(write(i2cbus, buffer, 3) != 3) {
      logMsg(logFile, "Unsuccessful cmd write to INA219", ERROR);	  
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

} 




int getBatteryVoltage() {
   char buffer[4];
   char conv[2];
   int config_reg;
   int result;
   int AD_sleep = 0;
      
#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, INA219_ADDR)) {
      logMsg(logFile, "Can't set ADC0 I2C address", ERROR);
   }
   usleep(I2C_DELAY);


   buffer[0] = (char) INA219_REG_VBUS;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful INA219_REG_VBUS write to ADC0", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 2) != 2) {
      logMsg(logFile, "Unsuccessful INA219_REG_VBUS read from ADC0", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

   result = (int) (( conv[0] << 5 ) | ((conv[1] >> 3) & 0xF)) & 0x1FFF;
   if(result<MIN_VBUS_RANGE) {
      logMsg(logFile, "Clipping out of range INA219_REG_VBUS", ERROR);	  
      result = MIN_VBUS_RANGE;
   }
   else if (result>MAX_VBUS_RANGE) {
      logMsg(logFile, "Clipping out of range INA219_REG_VBUS", ERROR);	  
      result = MAX_VBUS_RANGE;
   }
   result = MAX_VBAT_RANGE*result/MAX_VBUS_RANGE;   // mV
#else
   if(read(devRandom, conv, 1) != 1) {
      logMsg(logFile, "Unsuccessful read from /dev/random", ERROR);	   
   }
   result = (MAX_VBAT_RANGE * abs(conv[0]))/128;
#endif   
   return result;
} 

int getOperatingCurrent() {
   char buffer[4];
   char conv[2];
   int config_reg;
   int result;
   int current;
   int AD_sleep = 0;
      
#ifdef CCSR_PLATFORM
   pthread_mutex_lock(&semI2c);
   if(ioctl(i2cbus, I2C_SLAVE, INA219_ADDR)) {
      logMsg(logFile, "Can't set ADC0 I2C address", ERROR);
   }
   usleep(I2C_DELAY);


   buffer[0] = (char) INA219_REG_VSHUNT;
   if(write(i2cbus, buffer, 1) != 1) {
      logMsg(logFile, "Unsuccessful INA219_REG_VSHUNT write to ADC0", ERROR);	  
   }
   usleep(I2C_DELAY);
   if(read(i2cbus, conv, 2) != 2) {
      logMsg(logFile, "Unsuccessful INA219_REG_VSHUNT read from ADC0", ERROR);	   
   }
   usleep(I2C_DELAY);
   pthread_mutex_unlock(&semI2c);

   result = (int) (( conv[0] << 8 ) | (conv[1]  & 0xF)) & 0xFFFF;
   if(result<MIN_VSHUNT_RANGE) {
      logMsg(logFile, "Clipping out of range INA219_REG_VSHUNT", ERROR);	  
      result = MIN_VSHUNT_RANGE;
   }
   else if (result>MAX_VSHUNT_RANGE) {
      logMsg(logFile, "Clipping out of range INA219_REG_VSHUNT", ERROR);	  
      result = MAX_VSHUNT_RANGE;
   }
   result = MAX_I_RANGE*result/MAX_VSHUNT_RANGE;   // mA
#else
   if(read(devRandom, conv, 1) != 1) {
      logMsg(logFile, "Unsuccessful read from /dev/random", ERROR);	   
   }
   result = (MAX_I_RANGE * abs(conv[0]))/128;
#endif   
   return result;
} 




void *powerMonitor() {

   logMsg(logFile, "Starting Power Monitor", LOG); 

   int currHistory[CURRENT_AVG_WINDOW];
   int currHistoryPtr;
   int currAvg;
   int voltHistory[VOLTAGE_AVG_WINDOW];
   int voltHistoryPtr;
   int voltAvg;
   int n;
   int m;
 
   int x, y, voltMonDiv;
   
   x = CURR_MONITOR_PERIOD;
   y = CURR_MONITOR_PERIOD;
   voltMonDiv = VOLT_MONITOR_PERIOD/CURR_MONITOR_PERIOD;

   for(n=0;n<VOLTAGE_AVG_WINDOW;n++) {
      voltHistory[n] = 100;
   }
   for(n=0;n<CURRENT_AVG_WINDOW;n++) {
      currHistory[n] = 0;
   }
      
   currHistoryPtr = 0;
   voltHistoryPtr = 0;
   m = 0;
   
   while(1) {
      if(m>voltMonDiv) {
         // After 'voltMonDiv' current samples, do one voltage monitor
         ccsrState.batteryVoltage = getBatteryVoltage();
         ccsrState.batteryPercent = 100 * (ccsrState.batteryVoltage - MIN_VMAINBATTERY) / VMAINBATTERY_RANGE; 
         m = 0;


         voltHistory[voltHistoryPtr] = ccsrState.batteryPercent;
         if(voltHistoryPtr>VOLTAGE_AVG_WINDOW) {
            voltHistoryPtr = 0;
         }
         else {
            voltHistoryPtr = voltHistoryPtr + 1;
         }
      
         voltAvg = 0;
         for(n=0;n<VOLTAGE_AVG_WINDOW;n++) {
            voltAvg = voltAvg + voltHistory[n];
          }
         voltAvg = voltAvg/VOLTAGE_AVG_WINDOW;
         ccsrState.batteryPercent = voltAvg;
      
         if ((voltAvg < LOW_BATTERY_LEVEL) && (ccsrState.lowBattery == 0)) {
            // LowBattery is permanent: needs to be reset by cold reset,
	    // or later by battery charging function.
 	    ccsrState.lowBattery = 1;
 	    write(pipeSoundGen[IN], &sound[batteryAlarm], sizeof(sound[batteryAlarm]));
	    lcdEvent = EVENT_LOW_BATT;
 	    write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
 	    logMsg(logFile, "Low Battery event", LOG);
         }
      }
      else {
        m = m +1;
      }
      ccsrState.operatingCurrent = getOperatingCurrent();
      // Probably need log curve to make linear:

      currHistory[currHistoryPtr] = ccsrState.operatingCurrent;
      if(currHistoryPtr>CURRENT_AVG_WINDOW) {
         currHistoryPtr = 0;
      }
      else {
         currHistoryPtr = currHistoryPtr + 1;
      }
      
      currAvg = 0;
      for(n=0;n<CURRENT_AVG_WINDOW;n++) {
         currAvg = currAvg + currHistory[n];
      }
      currAvg = currAvg/CURRENT_AVG_WINDOW;
      ccsrState.operatingCurrent = currAvg;
      
      if ((currAvg > ccsrState.maxOperatingCurrent) && (ccsrState.currentLimit == 0)) {

         // CurrentLimit is only set here, needs to be reset by evasiveAction
	 ccsrState.currentLimit = 1; 
	  
 	 write(pipeSoundGen[IN], &sound[currentAlarm], sizeof(sound[currentAlarm]));
	 lcdEvent = EVENT_CURR_LIMIT;
 	 write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
// 	 logMsg(logFile, "Exceeding MAX operating current event", LOG);
      }
 
//     printf("sp2 %d sp2 %d lim %d curr %d\n", ccsrState.speedMotor1, ccsrState.speedMotor1, ccsrState.currentLimit, ccsrState.operatingCurrent);

//      printf("batTeryVoltage %d operatingCurrent %d currAvg %d max %d\n", ccsrState.batteryVoltage, ccsrState.operatingCurrent, currAvg, ccsrState.currentLimit);
//      printf("operatingCurrent %d\n", ccsrState.operatingCurrent);
//printf("percent %d %d\n", ccsrState.batteryPercent, (100*(ccsrState.batteryVoltage - MIN_VMAINBATTERY))/VMAINBATTERY_RANGE);
//printf("currentLimit %d\n", ccsrState.currentLimit);
//printf("lowBattery %d\n", ccsrState.lowBattery);
      usleep(CURR_MONITOR_PERIOD);
   }
}

