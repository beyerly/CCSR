#define MINNOW_LED1t "/sys/class/leds/minnow_led1/trigger"
#define MINNOW_LED1b "/sys/class/leds/minnow_led1/brightness"

#define BUTTON0_VAL "/sys/class/gpio/gpio0/value"
#define BUTTON1_VAL "/sys/class/gpio/gpio1/value"

#define GPIO244_DIR "/sys/class/gpio/gpio244/direction"
#define GPIO244_VAL "/sys/class/gpio/gpio244/value"
#define GPIO244_EDG "/sys/class/gpio/gpio244/edge"
#define GPIO245_DIR "/sys/class/gpio/gpio245/direction"
#define GPIO245_VAL "/sys/class/gpio/gpio245/value"
#define GPIO245_EDG "/sys/class/gpio/gpio245/edge"
#define GPIO246_DIR "/sys/class/gpio/gpio246/direction"
#define GPIO246_VAL "/sys/class/gpio/gpio246/value"
#define GPIO246_EDG "/sys/class/gpio/gpio246/edge"
#define GPIO247_DIR "/sys/class/gpio/gpio247/direction"
#define GPIO247_VAL "/sys/class/gpio/gpio247/value"
#define GPIO247_EDG "/sys/class/gpio/gpio247/edge"
#define GPIO248_DIR "/sys/class/gpio/gpio248/direction"
#define GPIO248_VAL "/sys/class/gpio/gpio248/value"
#define GPIO248_EDG "/sys/class/gpio/gpio248/edge"
#define GPIO249_DIR "/sys/class/gpio/gpio249/direction"
#define GPIO249_VAL "/sys/class/gpio/gpio249/value"
#define GPIO249_EDG "/sys/class/gpio/gpio249/edge"
#define GPIO250_DIR "/sys/class/gpio/gpio250/direction"
#define GPIO250_VAL "/sys/class/gpio/gpio250/value"
#define GPIO250_EDG "/sys/class/gpio/gpio250/edge"
#define GPIO251_DIR "/sys/class/gpio/gpio251/direction"
#define GPIO251_VAL "/sys/class/gpio/gpio251/value"
#define GPIO251_EDG "/sys/class/gpio/gpio251/edge"

#define I2C_BUS "/dev/i2c-0" 

//#ifdef CCSR_PLATFORM
#define LOG_FILE "/home/root/ccsr/ccsr.log"
//#else
//#define LOG_FILE "/nfs/fm/disks/fm_cse_00820/rdegruij/personal/ccsr/ccsr.log"
//#endif


// All times in usec
#define MSG_STRING_SIZE 50
#define BOOT_BUTTON_TIMEOUT -1 
#define BRAIN_LOOP_INTERVAL 50000 // 0.05 sec
#define BUTTON_HOLD_TIME 2000000/BRAIN_LOOP_INTERVAL
#define TERMINATION_DELAY 5000000
#define EVASIVE_ACTION_DELAY 2000000/BRAIN_LOOP_INTERVAL
#define MAX_EVASIVE_ACTION_DEPTH_BEFORE_REORIENTATION 2
#define DETOUR_LENGTH 3000000/BRAIN_LOOP_INTERVAL
#define BACK_UP_LENGTH 3000000/BRAIN_LOOP_INTERVAL
#define I2C_DELAY 500
// Seeed Ultrasonic sensor

#define PINGPULSEWIDTH 5      // useconds
#define PINGTIMEOUT 38000      // useconds
#define TIME_BETWEEN_PINGS 5   // seconds

#define NUM_AUDIO_SAMPLES 3



void ccsrTerminate();
void stateChange(char state);


enum enableType {OFF, ON};
enum turnDirType {RIGHT, LEFT};
enum turnType {NOSCAN, SCAN};
enum msgType {LOG, ERROR};
enum pipeDir {OUT, IN};
enum ccsrSMType {SM_RESET,                
                 SM_DIAGNOSTICS,
		 SM_ORIENTATION,
		 SM_DRIVE_TO_TARGET,
		 SM_POSITION_FOR_PICKUP,   
		 SM_PICKUP_OBJECT,        
		 SM_RETURN_TO_START_LOCATION,  
		 SM_EXPLORE,
		 SM_OBSERVE, 
		 SM_REMOTE_CONTROLLED,
		 SM_TURN_TO_LOCKED_OBJECT, // unused
		 SM_TRACK_AND_FOLLOW,      // unused
                 SM_SLEEP
		 };

#define NUM_BOTTONS 7 // one big red, 4 numpads, one PID

typedef struct ccsrStateType {
   char	 state;
   char	 state_next;
   char	 action;
   char  noMotors;
   char  ccsrPause;
   char  remoteControlled;
   
   // Locomotion
   int	 speedMotor1;
   int	 speedMotor2;
   int   minMotorTurnSpeed;
   int   minMotorSpeed;

   // Power management
   int	 batteryVoltage;
   int	 operatingCurrent;
   int   batteryPercent;
   int   lowBattery;
   int   currentLimit;
   int   maxOperatingCurrent;

   // Navigation
   int	 locationX;
   int	 locationY;

   // User interface
   char  button0Pressed; // big red one
   char  button1Pressed; // Pad #1
   char  button2Pressed; // Pad #2
   char  button3Pressed; // Pad #3
   char  button4Pressed; // Pad #4

   // Pan/Tilt
   int panPulseWidth;
   int tiltPulseWidth;
   int pan;
   int tilt;

   // Arm
   int armPulseWidth;
   int arm;
   int elbowPulseWidth;
   int elbow;
   int wristPulseWidth;
   int wrist;
   int handPulseWidth;
   int hand;

   int RPulseWidth;
   int GPulseWidth;
   int BPulseWidth;
   

   // Enviroment 
   int   temp;     // Celcius
   int   ambientLight;
   char  motionDetected; 
   char  noiseDetected; 
   int   audioPeakThreshold;
   long  audioEnergyThreshold;
   int   speakerVolume;

   // Proximity sensors
   int	 irDistFrontLeft;
   int	 irDistFrontRight;
   int	 irDistBelow;
   int	 irDistBack;
   int   proximity;

   // Sonar
   int	 sonarDistFront;
   
   // Navigation 
   int   heading;  // Degrees
   int   targetHeading;  // Degrees
   char  targetReached;
   int   compassCalibrationOffsetX;
   int   compassCalibrationOffsetY;
   int   compassRawFieldX;
   int   compassRawFieldY;
   int   evasiveActionCompensation;
   int   evasiveActionDepth;
   int   gyroAngMoment_X;   // Pitch
   int   gyroAngMoment_Y;   // Yaw (Turn)
   int   gyroAngMoment_Z;   // Roll
   int	 timeAtRest;


   // 360 degree profiles of sonar depth and ambient light, dead-ahead
   int profileValid[361];
   int ambientLightProfile[361];
   int sonarDistProfile[361];     
   int sonarDistDownFront;       // Detect precipice or steep hill
   
   // Sensory input enables
   char continuousLCDRefresh;
   char proximitySensorsOn;
   char sonarSensorsOn;
   char environmantalSensorsOn;
   char navigationOn;
   char gyroOn;
   char pidMotionDetectOn;
   char noiseDetectOn;
   char cameraCaptureOn;
   char continuousVoiceRecognitionOn;

   // Telemetry
   char telemetryConnected;
   
   
   // Timers and alarms
   unsigned long timer;
   unsigned long timerAlarm[3];
   
   // LCD display menues/modes
   char statusField;
   char minorMsgMode;
   char menueItem;
   
   short* audioMemory[NUM_AUDIO_SAMPLES];
   char   audioMemoryValid[NUM_AUDIO_SAMPLES];
   int    audioMemorylength[NUM_AUDIO_SAMPLES];
   char   activeAudioSample;

   // Opencv visual
   
   // Define target object CCSR is tracking in terms of HSV color range and volume (size)
   int targetColor_iLowH;  
   int targetColor_iHighH; 
   int targetColor_iLowS;  
   int targetColor_iHighS; 
   int targetColor_iLowV;  
   int targetColor_iHighV; 
   int targetColorVolume;
   char targetColorName[60];

   char visualProcessingOn;     // if 1, camera is used by visual.cpp, and capturing images
   char trackTargetColorOn;     // if set to 1 by user, CCSR tracks target object. 
   int  objectTracked;          // Set to 1 by *visual if target object is being tracked
   int  trackedObjectCentered;  // Set to 1 by *camtrack if tracked object is in the center of the captured image
  
   // Current coordinates and volume of tracked object, continuously updated by *visual
   double targetVisualObject_X;  
   double targetVisualObject_Y;  
   double targetVisualObject_Vol;  

   // Object analysis
   char analyzeObject;  // *visual thread will analyse object once if set to '1', and populate following 3 color values
   // Captured HSV values after analysis
   int analyzedObjectH;  
   int analyzedObjectS;  
   int analyzedObjectV;  


   // Mood/Emotions

   char randomEyeMovements;
   char blinkRate;
   char eyeMovementRate;
   char showEmotion;

   int happiness;
   int arousal;

   int   mood;
   int   fear;
   int   endorphine;
   int   adrenaline;


} ccsrStateType;

/*
ccsrState.continuousLCDRefresh;
ccsrState.proximitySensorsOn;
ccsrState.sonarSensorsOn;
ccsrState.environmantalSensorsOn;
ccsrState.navigationOn;
ccsrState.pidMotionDetectOn;
ccsrState.noiseDetectOn;
*/
