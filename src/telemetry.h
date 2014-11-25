void *cmdInterface_telemetry();
void *cmdInterface_NLP();
//void ccsrExecuteCmd(int n, int wfd);
void ccsrExecuteCmd(char **splitLine, int n, int wfd);

#define RFCOMM0 "/dev/rfcomm0"
#define NUM_CMD_ARG 5
#define PROFILE_DUMP "profile_dump.csv"
#define STATE_CSV_DUMP "ccsrState_dump.csv"
#define NLP_FIFO_IN   "nlp_fifo_in"
#define NLP_FIFO_OUT  "nlp_fifo_out"
#define MAX_STRING_LEN 100		  
#define MAX_TELEMETRY_STRING_LEN 1000		  

enum cmdInterfaces {CMDIF_NLP,
		    CMDIF_TELEMETRY,
		    NUM_CMDIF
		   };

typedef struct ccsrCmdStructType {

   int cmd;
   
   int arguments[NUM_CMD_ARG];

} ccsrCmdStructType;
		  
		  
