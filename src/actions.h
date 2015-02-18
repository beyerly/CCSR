
#define NOD_SHAKE_SPEED 95
#define NOD_SHAKE_DELAY 100


enum ccsrActionType {EVASIVE_ACTION,
   ORIENTATION,
   PAUSED,
   TURN_TO_TARGET_HEADING,
   CAP_PLAYB_ACTION,
   NO_ACTION,
   ANALYZING_OBJ,
   FINDING_OBJ
};

enum orientationModeType {FULL,
                          FORWARD_ONLY,
			  NUM_ORIENT_MODES};


int evasiveAction();
int orientation(int angle);
void actionPause();
void turnToTargetHeading(int scan);
void turnToTargetHeadingDirect(int scan, int turnDir);
int  sonarScan(int range);
void sonarScanDown();
void getMinimumTurnSpeed();
void turnAtMinPowerInPlace(int dir, int time);
void driveAtMinPower(int dir, int time);

void analyzeObject();
void findAndPickupObject();
void giveObjectAndFoldArm();

void initColors();
void extendArm();
void dropAndFoldArm();
void grabObjectFromFixedGroundLocation();

void goToSleep();
void wakeFromSleep();

void shakeNo();
void nodYes();

void grab0();
void drop0();

#define SONAR_SCAN_DELAY 40000
#define NUM_COLORS 16


// Struct assigning a name to an HSV color defined by a range of HSV values. 
// Used to lookup a color name based on HSV values
typedef struct colorType {
   int iLowH;
   int iHighH;

   int iLowS;
   int iHighS;

   int iLowV;
   int iHighV;

   char name[30];
} colorType;
