void configNav();
void *navigation();
int calibrateCompass();

#define NAV_SENSE_PERIOD 50000 // 0.05 sec
#define NAV_DUMP "nav_dump.csv"

#define LIN_ACC_ADDR 0x19
#define COMPASS_ADDR 0x1E
#define GYRO_ADDR    0x6B
#define BARO_ADDR    0x77

#define AUTO_INC 0x80

// Linear Accelerator
#define CTRL_REG1_A 0x20
#define OUT_X_H_A  0x29
#define OUT_X_L_A  0x28
#define OUT_Y_H_A  0x2B
#define OUT_Y_L_A  0x2A
#define OUT_Z_H_A  0x2D
#define OUT_Z_L_A  0x2C

#define ODR_offset 4
#define ZYX_EN_offset 0

#define ODR_NORMAL_50HZ 4
#define ODR_NORMAL_1HZ 1
#define Z1Y1X1 0x7 
 
// Gyro
#define TEMP_OUT_G 0x26
#define CTRL_REG1_G 0x20
#define OUT_X_H_G  0x29
#define OUT_X_L_G  0x28
#define OUT_Y_H_G  0x2B
#define OUT_Y_L_G  0x2A
#define OUT_Z_H_G  0x2D
#define OUT_Z_L_G  0x2C

#define G_PD_offset 3

#define ODR_NORMAL_95HZ125CO 0
#define G_PD_NORMAL 0x1 
 

// Magnetic 
#define CRA_REG_M 0x0
#define CRB_REG_M 0x1
#define MR_REG_M 0x2
#define OUT_X_H_M  0x3
#define OUT_X_L_M  0x4
#define OUT_Z_H_M  0x5
#define OUT_Z_L_M  0x6
#define OUT_Y_H_M  0x7
#define OUT_Y_L_M  0x8
#define SR_REG_Mg 0x9 
#define IRA_REG_M 0xA
#define IRB_REG_M 0xB
#define IRC_REG_M 0xC
#define TEMP_OUT_H_M 0x31
#define TEMP_OUT_L_M 0x32

#define TEMP_EN_offset 7
#define DO_offset 2
#define MD_offset 0

#define TEMP_EN 1
#define DO_15HZ 4
#define MD_CONT 0x0
