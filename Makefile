
#CC = /home/robert/minnowBuild3/setup-scripts/build/tmp-angstrom_v2013_12-eglibc/sysroots/x86_64-linux/usr/libexec/core2-angstrom-linux/gcc/i586-angstrom-linux/4.8.1/gcc -m32 -march=core2 -msse3 -mtune=generic -mfpmath=sse --sysroot=/home/robert/minnowBuild3/setup-scripts/build/tmp-angstrom_v2013_12-eglibc/sysroots/minnow
CC = /home/robert/minnowBuild3/setup-scripts/build/tmp-angstrom_v2013_12-eglibc/sysroots/x86_64-linux/usr/bin/core2-angstrom-linux/i586-angstrom-linux-gcc -m32 -march=core2 -msse3 -mtune=generic -mfpmath=sse --sysroot=/home/robert/minnowBuild3/setup-scripts/build/tmp-angstrom_v2013_12-eglibc/sysroots/minnow
CPP = /home/robert/minnowBuild3/setup-scripts/build/tmp-angstrom_v2013_12-eglibc/sysroots/x86_64-linux/usr/bin/core2-angstrom-linux/i586-angstrom-linux-g++ -m32 -march=core2 -msse3 -mtune=generic -mfpmath=sse --sysroot=/home/robert/minnowBuild3/setup-scripts/build/tmp-angstrom_v2013_12-eglibc/sysroots/minnow

# ./build/tmp-angstrom_v2013_12-eglibc/sysroots/x86_64-linux/usr/bin/core2-angstrom-linux/i586-angstrom-linux-gcc
# ./build/tmp-angstrom_v2013_12-eglibc/sysroots/x86_64-linux/usr/libexec/core2-angstrom-linux/gcc/i586-angstrom-linux/4.8.1/gcc
#LDFLAGS = -Wl,-L/nfs/fm/disks/fm_cse_00820/rdegruij/personal/portaudio/lib/ -Wl,-L/nfs/fm/disks/fm_cse_00820/rdegruij/personal/lib/
#LDFLAGS = -Wl,-L/usr/local/lib/

OPENCV_LINK_FLAGS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d \
                    -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_flann

ccsr_obj = ccsr.o utils.o motor.o irSensors.o sound.o mood.o vocal.o powerMonitor.o navigation.o lcdDisp.o \
          actions.o  telemetry.o servoCtrl.o  gpio.o visual.o
	  
	  
telccsr_obj = telccsr.o
headers = src/ccsr.h src/utils.h src/motor.h src/irSensors.h \
          src/sound.h src/mood.h src/vocal.h src/powerMonitor.h src/navigation.h \
	  src/lcdDisp.h src/actions.h  src/telemetry.h src/telccsr.h src/servoCtrl.h \
	  src/mjpg_streamer.h src/gpio.h src/visual.h

all: ccsr telccsr 

%.o : src/%.c $(headers)
	$(CC)   -O3 -DCCSR_PLATFORM -c -pthread  $< -o $@ 
#	$(CC)  -g -I /nfs/fm/disks/fm_cse_00820/rdegruij/personal/include -DCCSR_PLATFORM -c -pthread  $< -o $@ 
#	$(CC) -DDEBUG -c -pthread  $< -o $@ 

%.o : src/%.cpp $(headers)
	$(CPP) -O3  -DCCSR_PLATFORM -c  $< -o $@ 


ccsr: $(ccsr_obj) $(headers) 
	$(CPP) $(ccsr_obj) $(LDFLAGS) $(OPENCV_LINK_FLAGS) -O3 -lsndfile -lasound -lm -ldl -o ccsr -pthread -lportaudio -lespeak

ocv: src/test.cpp
	$(CPP) src/test.cpp $(OPENCV_LINK_FLAGS) -o opencvtest 

telccsr: $(telccsr_obj) $(headers) src/client.c
	$(CC) $(telccsr_obj) $(LDFLAGS) -g -lreadline -o telccsr 
	$(CC)  -g src/client.c -o client

clean:
	rm $(ccsr_obj); rm $(telccsr_obj); rm ccsr; rm telccsr; 


