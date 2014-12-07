
#define MAX_ARGS 100
#define MAX_ARG_LEN 60

#define CCSR_FIFO_IN   "fifo_in"
#define CCSR_FIFO_OUT  "fifo_out"
#define TEL_EOM "*"


enum telccsrcmd  {CMD_SET,
		  CMD_DUMP,
		  CMD_SET_CCSRSTATE,
		  CMD_ORIENTATION,
		  CMD_TURN_TO,
		  CMD_SAY,
		  CMD_QUIT,
		  CMD_TERM,
		  CMD_AUDIO_CAPTURE,
		  CMD_DIAGNOSTICS,
		  CMD_AUDIO_PLAYBACK,
		  CMD_CALIBRATE_COMPASS,
		  CMD_RESET,
		  CMD_DUMMY,
		  CMD_ANALYZE_OBJ,
		  CMD_FIND_OBJ,
		  CMD_GIVE_OBJ,
		  NUM_CMD
		  };

enum telccsrdumpsubcmd  {DUMPSUBCMD_ALL,
 			 DUMPSUBCMD_PROFILE,
 			 DUMPSUBCMD_CSV,
			 NUM_DUMPSUBCMD
		        };
enum telccsronoffsubcmd  {SUBCMD_OFF,
 			  SUBCMD_ON,
			  NUM_ONOFFSUBCMD
		         };

enum telccsrset          {REMOTE,
                          SENS_PROX,
 			  SENS_SONAR,
			  SENS_ENV,
			  SENS_NAV,
			  SENS_PID,
			  SENS_NOISE,
			  SENS_CAM,
			  SENS_ALL,
			  SPEED,
			  PANTILT,
			  TRACK_COLOR,
			  STATE,
			  SENS_GYRO,
			  MIN_TURN_SPEED,
			  MAX_OP_CURR,
			  ARM,
			  MOTORPRESCALER,
			  VOLUME,
			  LCDDISP,
			  TARGET_COLOR_VOLUME,
			  NUM_SENS_CMD
		         };

