#define MOTOR_ADDR 15 


#define SETPWMAB   0x82 
#define SETFREQ    0x84 
#define CHANGEADDR 0x83 
#define CHANNELSET 0xaa 
#define MOTOR1     0xa1 
#define MOTOR2     0xa5 

#define MOTOR_TURN_THRESHOLD 150
#define MOTOR_POS_THRESHOLD 100
#define MOTOR_NEG_THRESHOLD -30
#define MAX_MOTOR_SPEED 230 
#define MAX_MOTOR_TURNSPEED 250
#define SLOW_MOTOR_TURNSPEED 100  // Least motor power that still makes CCSR turn
#define SLOW_MOTOR_SPEED 60       // Least motor power that still moves CCSR fwd/back
#define MOTOR_SPEEDUP_STEPS 32 

#define SLOW_TURNSPEED_THRESHOLD 90  // deg

#define MOTOR_SPEED_CURVE_OFFSET MAX_MOTOR_SPEED 

#define SAVEADDR 'S' 
#define NOTSAVEADDR 'N' 
 
#define MOTOR_POWER_TO_SPEED_CONV 0

#define TARGET_HEADING_HYSTERESIS 10 // If we are driving within 10 degrees of target heading, we consider it on track
                                     // May need to lower hysteresis for accuracy

#define HEADING_QUEUE_DEPTH 4

enum motorDir {STATIONARY, FORWARD, REVERSE};

void setMotorspeed(int speed, unsigned char motor);
int motorDiagnostic();
int speedFiltered(int targetSpeed, int delta);

void setMotorPrescalerFrequency(int freq);
void setMotorPWM(int speedA, int speedB);
int initMotors();
void *driveToTargetHeading();
void *odometer();
