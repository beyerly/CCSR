


enum ccsrActionType {EVASIVE_ACTION,
 		     ORIENTATION,
		     PAUSED,
		     TURN_TO_TARGET_HEADING,
		     CAP_PLAYB_ACTION,
		     NO_ACTION};

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



#define SONAR_SCAN_DELAY 40000
#define NUM_COLORS 16

#define MAX_HUE         179
#define MAX_SATURATION  255
#define MAX_VALUE       255

#define HUE_WINDOW        20
#define SATURATION_WINDOW 40
#define VALUE_WINDOW      20

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
