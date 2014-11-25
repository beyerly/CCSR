#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <readline/history.h>
#include "telemetry.h"
#include "telccsr.h"

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <pthread.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>



int rfd, wfd;
char* line;
char in;
char eom[]  = TEL_EOM;
char fin[]  = CCSR_FIFO_IN;
char fout[] = CCSR_FIFO_OUT;
int c;

int main(){

   if ((wfd = open(fin, O_WRONLY)) < 0)
      perror("open() error for write end");
   if ((rfd = open(fout, O_RDONLY)) < 0)
      perror("open() error for read end");

   while(1) {
      line = readline("telccsr> ");  
      if(line != NULL) {
        add_history (line);
	write(wfd, line, strlen(line));
	write(wfd, eom, strlen(eom));
        // Read chars from CCSR fifo and dump on stdout untill it sends EOM
	read(rfd, &in, 1);
        while(in != 42) {
           printf("%c",in);
	   read(rfd, &in, 1);
        }
     }
  }
}
