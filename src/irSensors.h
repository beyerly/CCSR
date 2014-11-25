void configADC(int sensor);
int getIrDistance(int sensor);
void *proximitySensors();
void *environmentalSensors();
void *sonarSensors();


#ifdef CCSR_PLATFORM
#define IR_SENSE_PERIOD    50000 // 0.05 sec
#define SONAR_SENSE_PERIOD 500000 // 0.05 sec
#define ENV_SENSE_PERIOD   1000000 // 1 sec
#else
#define IR_SENSE_PERIOD 2000000 // 3 sec
#endif

// ADS1015 A/D converter

#define ADC0_ADDR 0x48
#define OS_offset 15
#define MUX_offset 12
#define PGA_offset 9
#define MODE_offset 8
#define DR_offset 5
#define COMP_MODE_offset 4
#define COMP_POLL_offset 3
#define COMP_Q_offset 0

#define OS_start_conv 1
#define OS_0 0
#define MUX_AIN0 4
#define MUX_AIN1 5
#define MUX_AIN2 6
#define MUX_AIN3 7
#define PGA_1x 1
#define MODE_SS 1 
#define MODE_CONT 0 
#define DR_3300 7
#define DR_1600 4
#define DR_128 0
#define COMP_MODE_TRAD 0
#define COMP_POLL_AL 0
#define COMP_Q_disable 3

#define ADC_REG_CONV 0
#define ADC_REG_CNFG 1
#define ADC_REG_THRS_LO 2
#define ADC_REG_THRS_HI 3

#define MAX_IR_RANGE 1536
#define MIN_IR_RANGE 30
#define MAX_IR_DIST_TO_DRIVE 350
#define QUANT_STEP 32 
#define IR_DIST_BOTTOM_THRESHOLD 1300     // If smaller than this, there's a drop in front
#define SONAR_DIST_FRONT_THRESHOLD 30
#define MAX_SONAR_RANGE 1536
#define MIN_SONAR_RANGE 30

// #define IR_SENSOR_EXPONENT 1.0025
#define IR_SENSOR_EXPONENT 1.003

// VNCL4000 light/proximity

#define VNCL4000_ADDR 0x13
#define REG0_CMD 0x80
#define IR_LED_CUR_SET 0x83
#define AMB_LIGHT_RES_H 0x85
#define AMB_LIGHT_RES_L 0x86
#define PROX_RES_H 0x87
#define PROX_RES_L 0x88

#define ALS_OD_offset 4
#define PROX_OD_offset 3

#define ALS_OD_START 1
#define PROX_OD_START 1

