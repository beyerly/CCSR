void *powerMonitor();
void configPowerMonitor();
int getBatteryVoltage();
int getOperatingCurrent();

#define CURR_MONITOR_PERIOD 10000 
#define VOLT_MONITOR_PERIOD 1000000 


// ADS1015 

#define INA219_ADDR 0x40
#define RST_offset 15
#define BRNG_offset 13
#define PG_offset 11
#define BADC_offset 7
#define CADC_offset 3
#define PMMODE_offset 0

#define RST_ASSERT 1
#define RST_DEASSERT 0
#define BRNG_32V 1
#define PG_DIV8 3
#define BADC_12b 3
#define CADC_12b 3
#define MODE_SHUNT_BUS_CONT 7



#define INA219_REG_CNFG 0
#define INA219_REG_VSHUNT 1
#define INA219_REG_VBUS 2
#define INA219_REG_PWR 3
#define INA219_REG_CUR 4
#define INA219_REG_CAL 5

#define MAX_VBUS_RANGE 8000
#define MIN_VBUS_RANGE 0
#define MAX_VBAT_RANGE 32000 // mV

#define MAX_VSHUNT_RANGE 32000
#define MIN_VSHUNT_RANGE 0
#define MAX_I_RANGE 3200 //mA

#define MIN_VMAINBATTERY 6000 // mV
#define MAX_VMAINBATTERY 8400 // mV
#define VMAINBATTERY_RANGE 2400 

#define MAX_OPERATING_CURRENT 2600 // mA
#define LOW_BATTERY_LEVEL 5 // percent
#define MAX_LOW_BATTERY_EVENTS 16 // So POWER_MONITOR_PERIOD*MAX_LOW_BATTERY_EVENTS delay before alarm
#define MAX_CURRENTLIMIT_EVENTS 16 // So POWER_MONITOR_PERIOD*MAX_CURRENTLIMIT_EVENTS delay before alarm

#define CURRENT_AVG_WINDOW 8
#define VOLTAGE_AVG_WINDOW 3
