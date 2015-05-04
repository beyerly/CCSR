

int logMsg(FILE* logFile, char* s, int type);
int logState(FILE* logFile, ccsrStateType state);
int bootCheckButtons();
int diagnostics();
int ccsrStateRest();
int ccsrSpeed();
void brainCycle();
int min(int a, int b);
int max(int a, int b); 
int ccsrSpeedDelta(int targetHeading);
int shortestTurnDir(int targetHeading);
int headingDelta(int targetHeading, int turnDir);
int deepestSonarDepthHeading();
int minLightHeading();
int maxLightHeading();
int addAngle(int angle, int hdng);
int addAngleToHeading(int angle);
int getPanToHeading(int targetHeading);
void triangulate(char beaconA, char beaconB, int headingA, int headingB, int* X, int* Y);
void initBeacons();



#define NUM_BEACONS 2