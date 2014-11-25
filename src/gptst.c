#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>


#include "ccsr.h"
#include "sound.h"
#include "telemetry.h"
#include "telccsr.h"
#include "lcdDisp.h"

ccsrStateType ccsrState;


// For Bluetooth
int rfcomm;
char BTConnectionEnabled;

ccsrCmdStructType ccsrCmdStruct;

char eom[]  = TEL_EOM;
char fin[]  = CCSR_FIFO_IN;
char fout[] = CCSR_FIFO_OUT;
char in;   
char line[100];
char i, n;


int rfd, wfd;
const char delimiters[] = " ";
char *token, *cp;
char splitLine[MAX_ARGS][MAX_ARG_LEN];
char i, x, y;
char cmd, subCmd, subSubCmd;
char string[100];
int value0, value1;



int main() {

		 printf("proximitySensorsOn %*d\n",5, ccsrState.proximitySensorsOn);
		 printf("sonarSensorsOn %*d\n",9, ccsrState.sonarSensorsOn);
		 printf("environmantalSensorsOn %*d\n", 1, ccsrState.environmantalSensorsOn);
		 printf("navigationOn %10d\n",ccsrState.navigationOn);
		 printf("pidMotionDetectOn %10d\n",ccsrState.pidMotionDetectOn);
		 printf("noiseDetectOn %10d\n",ccsrState.noiseDetectOn);
		 printf("cameraCaptureOn %10d\n",ccsrState.cameraCaptureOn);
		 printf("fear %10d\n",ccsrState.fear);
		 printf("ambientLight %10d\n",ccsrState.ambientLight);
		 printf("irDistFrontLeft %10d\n",ccsrState.irDistFrontLeft );
		 printf("irDistFrontRight %10d\n",ccsrState.irDistFrontRight);
		 printf("irDistBelow   %10d\n",ccsrState.irDistBelow   );
		 printf("irDistBack    %10d\n",ccsrState.irDistBack    );
		 printf("proximity     %10d\n",ccsrState.proximity     );
		 printf("sonarDistFront  %10d\n",ccsrState.sonarDistFront  );
		 printf("heading       %10d\n",ccsrState.heading	);

}
