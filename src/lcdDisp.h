
void lcdDisplayInit();
void lcdDisplayMsg(char *s1, char *s2);
void lcdDisplayStatus();
void lcdDisplayRefresh();
void *lcdRefresh();
void *lcdManager();

#define LCD_ADDR 0x28
#define LCDPREFIX 0xFE
#define LCD_CLEARSCREEN 0x51
#define LCD_SETCURSOR 0x45
#define LCD_NEXTLINE 0x40
#define LCD_CURSORHOME 0x46

#define LCD_REFRESH_PERIOD 100000 // 1 sec


enum lcdEventType {EVENT_NONE,
	           EVENT_LINUX_BOOTED,
                   EVENT_CCSR_STARTED,
                   EVENT_ACTION,
		   EVENT_TERMINATE,
		   EVENT_HOLD_TO_TERMINATE,
		   EVENT_SM_STATE_CHANGE,
		   EVENT_DISPLAY_STATUS,
		   EVENT_DISPLAY_MENUE,
		   EVENT_TOGGLE_MODE,
		   EVENT_CAPTURE_PLAYBACK,
		   EVENT_MOTION_DETECTED,
		   EVENT_NOISE_DETECTED,
		   EVENT_CURR_LIMIT,
		   EVENT_TELEMETRY_CONN,
		   EVENT_LOW_BATT,
		   EVENT_TRACKING_OBJECT,
		   EVENT_TARGET_LOCKED
		   };

enum lcdStatusField {fieldNone, 
                     fieldBattery, 
		     fieldHeading,
		     fieldtargetHeading, 
		     fieldTemp, 
	             fieldSonarDistFront, 
		     fieldAmbLight,
		     fieldFear,
		     numFields};

enum lcdMenueItem   {menueItemMotorsOn, 
                     menueItemSonarOn,
                     menueItemNavigationOn,
                     menueItemnvironmentalOn,
                     menueItemMotionDetectOn,
                     menueItemNoiseDetectOn,
		     
		     numMenueItems};

enum minorMsgMode  {SHOW_ACTION, 
                    SHOW_STATUS,
		    SHOW_EVENT,
		    SHOW_MENUE,
		    NUM_SHOW_MODES};
