
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/soundcard.h>
#include <espeak/speak_lib.h>
#include <fcntl.h>
#include <math.h>
#include <sndfile.h>
#include "ccsr.h"
#include "sound.h"
#include "facial.h"
#include "lcdDisp.h"
#include "mood.h"
#include <alsa/asoundlib.h>
    
FILE *txtFile;
char textFileGenerated;
char txtBuff[MAX_TXT_SIZE];

extern FILE *logFile;
extern ccsrStateType ccsrState;
extern int pipeSoundGen[2];
extern soundType sound[NUM_SOUNDS];
extern int pipeLCDMsg[2];
extern pthread_mutex_t semAudio;
extern int pipeFacialMsg[2];
expressionType expr;
char lcdEvent;

int x;

pthread_t   threadSoundCapture; 
int cont_cap_length;
int cont_cap_block_length;

short* cont_capture_buffer;

snd_pcm_t *playback_handle;
snd_pcm_t *parrot_handle;
snd_pcm_t *capture_handle;

char* captureBufferFlags;

char string[MSG_STRING_SIZE];

short* audio_buffer;
short wave_buffer_sine[WAV_BUF_SIZE];
short wave_buffer_square[WAV_BUF_SIZE];

#define PI 3.14159265


espeak_POSITION_TYPE position_type;
espeak_AUDIO_OUTPUT output;
char *path=NULL;
int Buflength = 500, Options=0;
void* user_data;
t_espeak_callback *SynthCallback;
espeak_PARAMETER Parm;
char Voice[] = {"lt+klatt2"};
unsigned int Size,position=0, end_position=0, flags=espeakCHARS_AUTO, *unique_identifier;
expressionType expr;


int writeSndFile(int numFrames, short* buffer) {
   // Set file settings, 16bit Mono PCM
   SF_INFO info;
   int fd;
   info.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
   info.channels = 1;
   info.samplerate = PLAYBACK_SPEED/WAV_DOWNSAMPLE_FACTOR;

   // Open sound file for writing
   printf("x1\n");
      if ((fd = open(VOICE_FILE, O_CREAT | O_WRONLY | O_TRUNC)) < 0) {
         perror("open() error for sndfile");
      }
   
//   SNDFILE *sndFile = sf_open(VOICE_FILE, SFM_WRITE, &info, 1);
   SNDFILE *sndFile = sf_open_fd(fd, SFM_WRITE, &info, SF_TRUE);
   if (sndFile == NULL) {
      fprintf(stderr, "Error opening sound file '%s': %s\n", VOICE_FILE, sf_strerror(sndFile));
      return -1;
   }

   // Write frames
   long writtenFrames = sf_writef_short(sndFile, buffer, numFrames);
   printf("writing wave file %s, %d frames\n", VOICE_FILE, writtenFrames);

   // Check correct number of frames saved
   if (writtenFrames != numFrames) {
      fprintf(stderr, "Did not write enough frames for source\n");
      sf_close(sndFile);
      return -1;
   }

   // Tidy up
   sf_write_sync(sndFile);
   sf_close(sndFile);
   return 0;
}   
   
void initEspeak() {
    output = AUDIO_OUTPUT_PLAYBACK;
    int I, Run = 1, L;    
    espeak_Initialize(output, Buflength, path, Options ); 
//    espeak_SetParameter(espeakVOLUME, 120, 0); 
    espeak_SetParameter(espeakPITCH, 95, 0); 
//    espeak_SetParameter(espeakRANGE, 80, 0); 
    //espeak_SetVoiceByName(Voice);
    const char *langNativeString = "german"; //Default to US English
/*    espeak_VOICE voice;
	memset(&voice, 0, sizeof(espeak_VOICE)); // Zero out the voice first
	voice.languages = langNativeString;
	voice.name = "english";
	voice.variant = 2;
	voice.gender = 1;
	voice.age = 10;
	espeak_SetVoiceByProperties(&voice);
*/}

void say(char *text) {
    Size = strlen(text)+1;
    expr.length = Size/4;
    expr.type = EXPR_TALK;
    write(pipeFacialMsg[IN], &expr,sizeof(expr)); 
    pthread_mutex_lock(&semAudio);
    espeak_Synth( text, Size, position, position_type, end_position, flags,
    unique_identifier, user_data );
    espeak_Synchronize( );
    pthread_mutex_unlock(&semAudio);
    while(ccsrState.talking) {
    usleep(10);
    }
}




void genSine(short* buffer, int size) {

   double step;
   int i;
      
   step = 2*PI/size;
   
   for(i=0; i<size; i++) {
      buffer[i]=(short) floor(((1 << 15) -1) * sin(i*step));
   }
}

void genSquare(short* buffer, int size) {
	int quart = size/2;
        int i;
	for (i=0;i<quart;i++){ 
	   buffer[i] = (short) floor(((1 << 15) -1) * i/quart);
	}
        for (i=quart;i<2*quart;i++) {
	   buffer[i] = buffer[2*quart - i];
	}
}

void set_playback_volume(long volume)
{
        long min, max, newvol;
        snd_mixer_t *handle = NULL;
        snd_mixer_selem_id_t *sid = NULL;
        const char *card = "default";
        const char *selem_name = "Master";
        float vpercent, fnewvol;

        if (snd_mixer_open(&handle, 0) < 0)
        {
                printf(" set_playback_volume() RPI - snd_mixer_open failed\n");
                fflush(stdout);
                perror("snd_mixer_open");
        }
        else
        {
                snd_mixer_attach(handle, card);
                snd_mixer_selem_register(handle, NULL, NULL);
                snd_mixer_load(handle);

                //snd_mixer_selem_id_alloca(&sid);
                snd_mixer_selem_id_malloc(&sid);
                snd_mixer_selem_id_set_index(sid, 0);
                snd_mixer_selem_id_set_name(sid, selem_name);
                snd_mixer_elem_t* elem = snd_mixer_find_selem(handle, sid);
		if (elem == NULL)
                {
                        printf(" set_playback_volume() RPI - snd_mixer_find_selem failed()\n");
                        fflush(stdout);
                }
                else
                {
                        snd_mixer_selem_get_playback_volume_range(elem, &min, &max);
			vpercent = (float)volume/100;
                        fnewvol = (float)min + vpercent * ( (float)max - (float)min);
                        newvol = (long)fnewvol;

//                      printf("playback volume=%ld  min=%ld   max=%ld   newvol=%ld\n",volume,min,max,newvol); fflush(stdout);
                        if ( snd_mixer_selem_set_playback_volume_all(elem,newvol) <0)
                        {
                                printf(" set_playback_volume() RPI - snd_mixer_selem_set_playback_volume_all() failed\n");
                                fflush(stdout);
                        }
                }
                snd_mixer_close(handle);
        }
        ccsrState.speakerVolume = volume;
}

void set_capture_volume(long volume)
{
   long min, max, newvol;
   snd_mixer_t *handle = NULL;
   snd_mixer_selem_id_t *sid = NULL;
   const char *card = "default";
   const char *selem_name = "Capture";
   float vpercent, fnewvol;
   if (snd_mixer_open(&handle, 0) < 0)
   {
   	   printf(" set_capture_volume() RPI - snd_mixer_open failed\n");
   	   fflush(stdout);
   	   perror("snd_mixer_open");
   }
   else
   {
       snd_mixer_attach(handle, card);
       snd_mixer_selem_register(handle, NULL, NULL);
       snd_mixer_load(handle);

       //snd_mixer_selem_id_alloca(&sid);
       snd_mixer_selem_id_malloc(&sid);
       snd_mixer_selem_id_set_index(sid, 0);
       snd_mixer_selem_id_set_name(sid, selem_name);
       snd_mixer_elem_t* elem = snd_mixer_find_selem(handle, sid);
       if (elem == NULL)
       {
   	       printf(" set_capture_volume() RPI - snd_mixer_find_selem failed()\n");
   	       fflush(stdout);
       }
       else
       {
   	  snd_mixer_selem_channel_id_t chn;
   	  for (chn = 0; chn <= SND_MIXER_SCHN_LAST; chn++)
   	  {
   	     int junk;
   	     const char name[100];
   	     // Minnowboard only has front-kleft and front-right channels.
   	     if (snd_mixer_selem_has_capture_channel(elem, chn))
   	     {
   		/*
   		printf("channel name %s\n", snd_mixer_selem_channel_name(chn));
   		printf("snd_mixer_selem_has_capture_channel %d\n", snd_mixer_selem_has_capture_channel(elem, chn));
   		printf("snd_mixer_selem_is_capture_mono %d\n", snd_mixer_selem_is_capture_mono(elem));
   		printf("snd_mixer_selem_has_common_volume %d\n", snd_mixer_selem_has_common_volume(elem));
   		printf("snd_mixer_selem_has_capture_volume %d\n", snd_mixer_selem_has_capture_volume(elem));
   		printf("snd_mixer_selem_has_capture_volume_joined %d\n", snd_mixer_selem_has_capture_volume_joined(elem));
   		printf("snd_mixer_selem_has_common_switch %d\n", snd_mixer_selem_has_common_switch(elem));
   		printf("snd_mixer_selem_has_capture_switch %d\n", snd_mixer_selem_has_capture_switch(elem));
   		printf("snd_mixer_selem_get_capture_switch %d\n", snd_mixer_selem_get_capture_switch(elem, chn, &junk));
   		printf("capture switch %d\n", junk);
   		printf("snd_mixer_selem_get_capture_volume %d\n", snd_mixer_selem_get_capture_volume(elem, chn, &newvol));
   		printf("capture volume %d\n", newvol);
   		printf("snd_mixer_selem_get_capture_switch %d\n", snd_mixer_selem_get_capture_switch(elem, chn, &junk));
   		printf("capture switch %d\n", junk);
   		*/
   		// Unmute Channel
   		snd_mixer_selem_set_capture_switch(elem, chn, 1);
   	     }
   	  }
   	  snd_mixer_selem_get_capture_volume_range(elem, &min, &max);
   	  vpercent = (float)volume/100;
   	  fnewvol = (float)min + vpercent * ( (float)max - (float)min);
   	  newvol = (long)fnewvol;
   	  // printf("volume=%ld  min=%ld   max=%ld   newvol=%ld\n",volume,min,max,newvol); fflush(stdout);
   	  if ( snd_mixer_selem_set_capture_volume_all(elem,newvol) <0)
   	  {
   	      printf(" set_playback_volume() RPI - snd_mixer_selem_set_playback_volume_all() failed\n");
   	      fflush(stdout);
   	  }
   	  //printf("snd_mixer_selem_get_capture_volume %d\n", snd_mixer_selem_get_capture_volume(elem, chn, &newvol));
       }  
       snd_mixer_close(handle);
    }
}



void initSounds(){
   sound[singleA].freq[0] = 440;
   sound[singleA].length[0] = 300;
   sound[singleA].vol[0] = 1000;
   sound[singleA].style = sine;
   sound[singleA].notes = 1;

   sound[doubleA].freq[0] = 440;
   sound[doubleA].length[0] = 200;
   sound[doubleA].vol[0] = 1000;
   sound[doubleA].freq[1] = 440;
   sound[doubleA].length[1] = 200;
   sound[doubleA].vol[1] = 1000;
   sound[doubleA].style = sine;
   sound[doubleA].notes = 2;

   sound[observeSnd].freq[0] = NOTE_F6;
   sound[observeSnd].length[0] = 600;
   sound[observeSnd].vol[0] = 1000;
   sound[observeSnd].freq[1] = NOTE_A5;
   sound[observeSnd].length[1] = 300;
   sound[observeSnd].vol[1] = 1000;
   sound[observeSnd].freq[2] = NOTE_C6;
   sound[observeSnd].length[2] = 300;
   sound[observeSnd].vol[2] = 1000;
   sound[observeSnd].freq[2] = NOTE_F6;
   sound[observeSnd].length[2] = 300;
   sound[observeSnd].vol[2] = 1000;
   sound[observeSnd].style = sine;
   sound[observeSnd].notes = 4;

   sound[orientationSnd].freq[0] = NOTE_F5;
   sound[orientationSnd].length[0] = 300;
   sound[orientationSnd].vol[0] = 1000;
   sound[orientationSnd].freq[1] = NOTE_F5;
   sound[orientationSnd].length[1] = 300;
   sound[orientationSnd].vol[1] = 1000;
   sound[orientationSnd].freq[2] = NOTE_A5;
   sound[orientationSnd].length[2] = 300;
   sound[orientationSnd].vol[2] = 1000;
   sound[orientationSnd].style = sine;
   sound[orientationSnd].notes = 3;

   sound[evasiveActionSnd].freq[0] = NOTE_G7;
   sound[evasiveActionSnd].length[0] = 300;
   sound[evasiveActionSnd].vol[0] = 1000;
   sound[evasiveActionSnd].freq[1] = NOTE_G8;
   sound[evasiveActionSnd].length[1] = 300;
   sound[evasiveActionSnd].vol[1] = 1000;
   sound[evasiveActionSnd].freq[0] = NOTE_G7;
   sound[evasiveActionSnd].length[0] = 300;
   sound[evasiveActionSnd].vol[0] = 1000;
   sound[evasiveActionSnd].freq[1] = NOTE_G8;
   sound[evasiveActionSnd].length[1] = 300;
   sound[evasiveActionSnd].vol[1] = 1000;
   sound[evasiveActionSnd].style = sine;
   sound[evasiveActionSnd].notes = 4;

   sound[batteryAlarm].freq[0] = NOTE_A5;
   sound[batteryAlarm].length[0] = 500;
   sound[batteryAlarm].vol[0] = 1000;
   sound[batteryAlarm].freq[1] = NOTE_C6;
   sound[batteryAlarm].length[1] = 500;
   sound[batteryAlarm].vol[1] = 0;
   sound[batteryAlarm].freq[2] = NOTE_F5;
   sound[batteryAlarm].length[2] = 500;
   sound[batteryAlarm].vol[2] = 1000;
   sound[batteryAlarm].style = sine;
   sound[batteryAlarm].notes = 3;

   sound[currentAlarm].freq[0] = NOTE_G7;
   sound[currentAlarm].length[0] = 200;
   sound[currentAlarm].vol[0] = 1000;
   sound[currentAlarm].freq[1] = NOTE_G6;
   sound[currentAlarm].length[1] = 200;
   sound[currentAlarm].vol[1] = 0;
   sound[currentAlarm].freq[2] = NOTE_G7;
   sound[currentAlarm].length[2] = 200;
   sound[currentAlarm].vol[2] = 1000;
   sound[currentAlarm].freq[3] = NOTE_G6;
   sound[currentAlarm].length[3] = 200;
   sound[currentAlarm].vol[3] = 1000;
   sound[currentAlarm].style = square;
   sound[currentAlarm].notes = 4;

   sound[linuxBooted].freq[0] = NOTE_Cs5;
   sound[linuxBooted].length[0] = 30;
   sound[linuxBooted].vol[0] = 1000;
   sound[linuxBooted].freq[1] = NOTE_Cs5;
   sound[linuxBooted].length[1] = 30;
   sound[linuxBooted].vol[1] = 1000;
   sound[linuxBooted].freq[2] = NOTE_A5;
   sound[linuxBooted].length[2] = 30;
   sound[linuxBooted].vol[2] = 1000;
   sound[linuxBooted].freq[3] = NOTE_F6;
   sound[linuxBooted].length[3] = 30;
   sound[linuxBooted].vol[3] = 1000;
   sound[linuxBooted].freq[4] = NOTE_G6;
   sound[linuxBooted].length[4] = 30;
   sound[linuxBooted].vol[4] = 1000;
   sound[linuxBooted].style = sine;
   sound[linuxBooted].notes = 5;

   sound[terminate].freq[0] =  NOTE_G6;
   sound[terminate].length[0] = 500;
   sound[terminate].vol[0] = 1000;
   sound[terminate].freq[1] = NOTE_F6;
   sound[terminate].length[1] = 500;
   sound[terminate].vol[1] = 1000;
   sound[terminate].freq[2] = NOTE_A5;
   sound[terminate].length[2] = 500;
   sound[terminate].vol[2] = 1000;
   sound[terminate].freq[3] = NOTE_E5;
   sound[terminate].length[3] = 500;
   sound[terminate].vol[3] = 1000;
   sound[terminate].freq[4] =NOTE_Cs5;
   sound[terminate].length[4] = 500;
   sound[terminate].vol[4] = 1000;
   sound[terminate].style = sine;
   sound[terminate].notes = 5;

   sound[keyPressed].freq[0] = NOTE_G6;
   sound[keyPressed].length[0] = 300;
   sound[keyPressed].vol[0] = 1000;
   sound[keyPressed].style = sine;
   sound[keyPressed].notes = 1;


   genSine(wave_buffer_sine, WAV_BUF_SIZE);
   genSquare(wave_buffer_square, WAV_BUF_SIZE);

   set_playback_volume(50);
   set_capture_volume(75);
   recordWave(DETECT_NOISE_LEVEL, 1000);

}

int initAudio(snd_pcm_t **pcm_handle, int mode) {

   int format;
   int samples;
   int n, i, step;
   int err;
   int speed = PLAYBACK_SPEED;
   int audioAvailable;
   

   snd_pcm_hw_params_t *hw_params;
   audioAvailable = 0;
/*   while(!audioAvailable) {
      // Wait untill espeak is done
      err = snd_pcm_open (pcm_handle, "hw:0,0", mode,  SND_PCM_NONBLOCK);
      if(err > 0) {
         audioAvailable = 1;
      }
      else {   
         usleep(500);
         printf("error %d", err);
  	 fprintf (stderr, "cannot open audio device %s (%s)\n", 
  		   "hw:0,0",
  		   snd_strerror (err));
      }
   }
*/
//   while(snd_pcm_open (pcm_handle, "hw:0,0", mode, SND_PCM_NONBLOCK) < 0) {
   while(snd_pcm_open (pcm_handle, "hw:0,0", mode, 0) < 0) {
	   usleep(300);
   }
/*  	  fprintf (stderr, "cannot open audio device %s (%s)\n", 
  		   "hw:0,0",
  		   snd_strerror (err));
  	  exit (1);
   }
*/     
   if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
  	  fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
  		   snd_strerror (err));
  	  exit (1);
   }
  		   
   if ((err = snd_pcm_hw_params_any (*pcm_handle, hw_params)) < 0) {
  	  fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
  		   snd_strerror (err));
  	  exit (1);
   }

   if ((err = snd_pcm_hw_params_set_access (*pcm_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
  	  fprintf (stderr, "cannot set access type (%s)\n",
  		   snd_strerror (err));
  	  exit (1);
   }

   if ((err = snd_pcm_hw_params_set_format (*pcm_handle, hw_params, SND_PCM_FORMAT_S16)) < 0) {
  	  fprintf (stderr, "cannot set sample format (%s)\n",
  		   snd_strerror (err));
  	  exit (1);
   }

  // Note: Minnowboard has (min, max) speed = (44100, 192000)
   if ((err = snd_pcm_hw_params_set_rate_near (*pcm_handle, hw_params, &speed, 0)) < 0) {
  	  fprintf (stderr, "cannot set sample rate (%s)\n",
  		   snd_strerror (err));
  	  exit (1);
   }


   if ((err = snd_pcm_hw_params_set_channels (*pcm_handle, hw_params, 2)) < 0) {
  	  fprintf (stderr, "cannot set channel count (%s)\n",
  		   snd_strerror (err));
  	  exit (1);
   }

   if ((err = snd_pcm_hw_params (*pcm_handle, hw_params)) < 0) {
  	  fprintf (stderr, "cannot set parameters (%s)\n",
  		   snd_strerror (err));
  	  exit (1);
   }

   snd_pcm_hw_params_free (hw_params);

   if ((err = snd_pcm_prepare (*pcm_handle)) < 0) {
  	  fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
  		   snd_strerror (err));
  	  exit (1);
   }


}

// length in ms
int playWave(int freq, int length, int vol, int style) {

   int format;
   int samples, faded_samples;
   int n, i, step, left, right;
   int err;
   int detune = 0;
   int fade = 0;
   int attennuate = vol;

   short* wb;

   switch(style) {
      case sine: 
         wb = wave_buffer_sine;
	 break;
      case sineFade: 
         wb = wave_buffer_sine;
	 fade = 1;
	 break;
      case square: 
         wb = wave_buffer_square;
	 break;
      case squareFade: 
         wb = wave_buffer_square;
	 fade = 1;
	 break;
      case sineDetune: 
         wb = wave_buffer_sine;
	 break;
      default:
         wb = wave_buffer_sine;
   }

   // Generate stereo wave: duplicate for left/right. CCSR currently only has speaker right channel

   samples = 2*length*PLAYBACK_SPEED/1000;
   faded_samples = 2*FADE_TIME*PLAYBACK_SPEED/1000;
   if(samples<faded_samples){
     faded_samples = samples;
  }
   audio_buffer = (short*) malloc(samples *sizeof(short));
   
   
   step = freq/(PLAYBACK_SPEED/WAV_BUF_SIZE);
   n = 0;
   for(i=0;i<samples;i=i+2) {
      right=i;
      left=i+1;
      audio_buffer[left] = (short) attennuate*wb[n]/MAX_FADE_VOL;
      audio_buffer[right] = (short) attennuate*wb[n]/MAX_FADE_VOL;
      n = n + step;
      if(i>samples-faded_samples) {
         attennuate = vol - ((i-(samples-faded_samples))*vol)/faded_samples ; 
      }
      if (n>=WAV_BUF_SIZE) {
         n = n - WAV_BUF_SIZE;
      }
   } 
   if(snd_pcm_writei (playback_handle, audio_buffer, samples/2) != samples) {
      logMsg(logFile, "Unsuccessful write of audio buffer", ERROR);	   
   }
   free(audio_buffer);


}

// capture frames from audio device. Interleaved stereo. Return capture audio
// buffer and energy/peak levels for left channel
void captureAudio(int* peak, long* energy, short* buffer, int frames) {
   
   int x; 
   short* buf_left;
   int samples;
   
   samples = 2*frames;
   
   initAudio(&capture_handle, SND_PCM_STREAM_CAPTURE);
   if(snd_pcm_readi (capture_handle, buffer, frames) != frames) {
      fprintf (stderr, "Unsuccessful aurio capture \n");
   }
   snd_pcm_close (capture_handle);

   // snd_pcm_readi records interleaved: even buffer enties are Right, odd buffer entries are Left. 
   // Current CCSR mic-amp jack plug is mono: only driving Left, so only odd buffer entries contain captured data!
   // T.b.d: map only mono capture to save space.

   // Calculate peak and energy for noise detection.
   *peak = 0;
   *energy = 0;
   // Analyse Left channel only

   buf_left = buffer ;
   for (x=0;x<samples;x=x+2*ENERGY_DOWNSAMPLE_FACTOR) {
	   if(abs(buf_left[x]) > *peak) {
         *peak = buf_left[x];
      }
      *energy = *energy + abs(buf_left[x]); 
   }
    *energy = *energy*ENERGY_DOWNSAMPLE_FACTOR/frames;
}

// record an audio wave, do with it as mode specifies
// length in ms
void recordWave(int mode, int length) {
   
   int x, y; 
   int peak;
   long energy;
   int samples;
   int frames;
   short* capture_buffer;
   short* capture_buffer_mono;

   
   frames = length*PLAYBACK_SPEED/1000;
   samples = 2*frames;

   // CCSR mic amplifier runs on separate 3v battery to prevent noise
   // WE use a GPIO pin to turn on power supply to mic amp through
   // a solid state relay.
   powerMicAmp(ON);

   capture_buffer = (short*) malloc(samples *sizeof(short));
   pthread_mutex_lock(&semAudio);
   captureAudio(&peak, &energy, capture_buffer, frames);

   switch (mode){
      case DETECT_NOISE_LEVEL: 
         ccsrState.audioPeakThreshold = peak;
         ccsrState.audioEnergyThreshold = energy;
         printf("Noise levels: peak %d energy %d\n", peak, energy);
      break;
      case LISTEN: 
         // Analysing Left channel only
         printf("Captured audio levels: peak %d energy %d\n", peak, energy);
//         if (peak > ccsrState.audioPeakThreshold + AUDIO_PEAK_HYSTRESIS){
//            printf("Audio Capture Peak trigger\n");
//         }
         if (energy > ccsrState.audioEnergyThreshold  + 10000){
            ccsrState.noiseDetected = 1;
	    lcdEvent = EVENT_NOISE_DETECTED;
 	    write(pipeLCDMsg[IN], &lcdEvent, sizeof(lcdEvent));
	    printf("Audio Capture energy trigger\n");
         }
	 else {
	    ccsrState.noiseDetected = 0;
	 }
      break;
      case PLAYBACK: 
         // CCSR seems to only have even buffer entries: so right channel. Need to switch speaker channel to left, where mic is?
         initAudio(&parrot_handle, SND_PCM_STREAM_PLAYBACK);
         if(snd_pcm_writei (parrot_handle, capture_buffer+1, samples/2) != samples) {
            logMsg(logFile, "Unsuccessful write of audio buffer", ERROR);	   
         }
         snd_pcm_close (parrot_handle);
      break;
      case CAPTURE: 
         // Capture audio snippets, overwrite circular buffer
	 if(ccsrState.audioMemoryValid[ccsrState.activeAudioSample]) {
	    free(ccsrState.audioMemory[ccsrState.activeAudioSample]);
	 }
         ccsrState.audioMemory[ccsrState.activeAudioSample] = (short*) malloc(samples *sizeof(short));
	 memcpy(ccsrState.audioMemory[ccsrState.activeAudioSample], capture_buffer, samples *sizeof(short));
         ccsrState.audioMemoryValid[ccsrState.activeAudioSample] = 1;
         ccsrState.audioMemorylength[ccsrState.activeAudioSample] = samples;
         ccsrState.activeAudioSample = ccsrState.activeAudioSample + 1;
	 if(ccsrState.activeAudioSample == NUM_AUDIO_SAMPLES) {
	    ccsrState.activeAudioSample = 0;
	 }
	 printf("acap: %d\n", ccsrState.activeAudioSample);
      break;
      case VOICE_RECOGNITION: 
         // Signal end of audio capture
         write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[singleA]));
         // usleep(500); // do we need any delay here?
         // Extract mono signal from left channel, and downsample from 44.1kHz to limit the wav file-size and reduce
	 // upload time to googleSpeech2textAPI
         capture_buffer_mono = (short*) malloc((frames/WAV_DOWNSAMPLE_FACTOR) *sizeof(short));
         x=0;
	 for(y=0;y<samples;y=y+2*WAV_DOWNSAMPLE_FACTOR){
	    capture_buffer_mono[x] = capture_buffer[y];
	    x=x+1;
	 }
         // Write audio file to disk: GoogleVoiceToText.sh deamon is 
	 // waiting for this file to appear, and will post it to
	 // Google voice to text API. The returned json file will be 
	 // parsed by CCSR_NLP.py, which will call ccsr back through
	 // a telemetry channel. GoogleVoiceToText.sh deamon will delete
	 // voice file after use, and resum polling
	 writeSndFile((frames/WAV_DOWNSAMPLE_FACTOR), capture_buffer_mono);
	 free(capture_buffer_mono);
      break;
      }
   pthread_mutex_unlock(&semAudio);
   powerMicAmp(OFF);
   free(capture_buffer);
}


int playAudioMemory(int recording) {
   
   if(ccsrState.audioMemoryValid[recording]) {
      printf("Error: no such recording\n");
      return 0;
   }
   else {
      pthread_mutex_lock(&semAudio);
      // CCSR seems to only have even buffer entries: so right channel. Need to switch speaker channel to left, where mic is?
      initAudio(&parrot_handle, SND_PCM_STREAM_PLAYBACK);
      if(snd_pcm_writei (parrot_handle, ccsrState.audioMemory[recording]+1, ccsrState.audioMemorylength[recording]/2) != ccsrState.audioMemorylength[recording]) {
         logMsg(logFile, "Unsuccessful write of audio buffer", ERROR);
      }
      snd_pcm_close (parrot_handle);
      pthread_mutex_unlock(&semAudio);
      return 1;
   }
}

void drainAudioPlayback() {
   
   if(snd_pcm_wait(parrot_handle, 10000) &&
      snd_pcm_wait(playback_handle, 10000)) {
      
   }	
   else {
      printf("wait\n");
      logMsg(logFile, "Waiting on audio buffer drain timed out", ERROR);	   
   }

}

// Pthread that continuously captures audio in a circular buffer. This is used to continuously listen for voice, and detect when a 
// full sentence has been captured. Total bufferspace is divided into blocks, and
// after capturing each block we toggle a flag in an array indicatign back to the calling process a block is ready for analysis.
// There is no sync back to this process, so we must assume the analysis process is always faster, which it should because we 
// significantly downsample when analyzing a block. When a sentence is detected and captured, this pthread is killed, it will be
// re-spawn when we start listenign for the next sentence.
void *soundCapture() {

   int x;
   int blocks;
   int wrPtr; 
   int samples;
   int frames, block_frames;
   short* index;

   int flagIdx;
   int result;
   
   frames       = cont_cap_length*PLAYBACK_SPEED/1000;          // total circular buffer space
   block_frames = cont_cap_block_length*PLAYBACK_SPEED/1000;    // capture block
   samples      = 2*frames;                                     // We sampel stereo (must change this!)
   wrPtr        = 0;
   flagIdx      = 0;
   blocks       = cont_cap_length/cont_cap_block_length;        // total number of block in circular buffer

   // Apparently, the first audio capture after opening the ALSA sound device contains excessive noise (maybe a 
   // 'pop' when enabling audio?), so we discard this first block.
   index = (short*) cont_capture_buffer + wrPtr;
   result = snd_pcm_readi(capture_handle, index, block_frames);
   if( result != block_frames) {
      printf("err %s\n",  snd_strerror(result));
      fprintf (stderr, "Unsuccessful aurio capture \n");
   }

   printf("Starting continuous audio capture using %d blocks \n", blocks);
   while(ccsrState.continuousVoiceRecognitionOn){
     index = (short*) cont_capture_buffer + wrPtr;
     // read a block
     result = snd_pcm_readi(capture_handle, index, block_frames);
      if( result != block_frames) {
         printf("err %s\n",  snd_strerror(result));
         fprintf (stderr, "Unsuccessful aurio capture \n");
      }
      // Set flag for this block to notify consumer. This is 2-phase handshake,
      // so we simply toggle everytime a block is ready
      if(captureBufferFlags[flagIdx] == 0) {
         captureBufferFlags[flagIdx] = 1;
      }
      else {
         captureBufferFlags[flagIdx] = 0;
      }
      // Move on to next block
      flagIdx = flagIdx + 1;
      if(flagIdx >= blocks) {
         // Wrap circular buffer
	 flagIdx = 0;
      }
      wrPtr = wrPtr + 2*block_frames;
      if (wrPtr>=samples) {
         // Wrap circular buffer
         wrPtr = 0;
      }
   }
}



// Pthread serving as robot ears. We continuously loop to see if audio capture needs to be done.
// Currently there's 2 modes: 
// - noise-detection simply triggers on heading any sound
// - continuousVoiceRecognition captures audio in a circular buffer, detects when a sentence has been spoken and writes
//   this sentece to an audio file for speech2text and NLP.
void *ears() {

   int x, y, p, q;
   int phraseStart, phraseEnd, phraseSize;
   int rdPtr; 
   int samples;
   int frames, block_frames;
   int xAvgWindow;
   short* capture_buffer_mono;
   int flagIdx;
   int flagPolarity;      
   short* index;
   int soundDetected;
   int silenceCount;
   int acc, avg;
   int blocks;

   cont_cap_length       = 10000;  // 10 sec total circular audio buffer. If we exceed that, we get garbled sentences, 
                                   // tbd: detect overflow and state 'keep sentences short' 
   cont_cap_block_length = 500;    //0.5 sec blocks
   blocks                = cont_cap_length/cont_cap_block_length;
   frames                = cont_cap_length*PLAYBACK_SPEED/1000;
   block_frames          = cont_cap_block_length*PLAYBACK_SPEED/1000;
   samples               = 2*frames;

   while(1) {
      usleep(EAR_PERIOD);
      if(ccsrState.noiseDetectOn) {
         recordWave(LISTEN, 500);
	 printf("ear %d\n", ccsrState.noiseDetected);
      }
      else if(ccsrState.continuousVoiceRecognitionOn) {
         printf("Starting continous Voice recognition using %d blocks of %d ms\n", blocks, cont_cap_block_length);	
	 write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[doubleA]));
         rdPtr = 0;
	 flagIdx = 0;
	 flagPolarity = 0;
	 soundDetected = 0;
	 silenceCount  = 0;
  	 // Allocate circular audio buffer and flag array
	 cont_capture_buffer = (short*) calloc(samples, sizeof(short));
 	 captureBufferFlags = (char*) calloc(blocks, sizeof(char));
	 // CCSR mic amplifier runs on separate 3v battery to prevent noise
	 // WE use a GPIO pin to turn on power supply to mic amp through
	 // a solid state relay.
	 powerMicAmp(ON);
         pthread_mutex_lock(&semAudio);                      // Clain audio device 
         initAudio(&capture_handle, SND_PCM_STREAM_CAPTURE); // Open audio device
 	 // Spawn pthread that continuously captures audio into circular buffer
	 if(pthread_create(&threadSoundCapture, NULL, soundCapture, NULL )) {
 	   logMsg(logFile, "Pthread can't be created", ERROR);
 	 }
         while(ccsrState.continuousVoiceRecognitionOn) {
	    acc=0;
	    // Wait for a block to become available
 	    while(captureBufferFlags[flagIdx] == flagPolarity) {
 	       usleep(200);
 	    }
	    // We got a block, downsample and calculate average, rectified sound level. 
	    // Analyse Left channel (even samples) only, right is not connected in CCSR
	    for (x=rdPtr;x< rdPtr + 2*block_frames;x=x+2*ENERGY_DOWNSAMPLE_FACTOR) {
	       acc = acc + abs(cont_capture_buffer[x]);
	    }
	    avg = acc*ENERGY_DOWNSAMPLE_FACTOR/block_frames;
	    if (avg>NOISE_LEVEL) {
 	       // This block may contain valid voice phrase. If we previously had empty (silent) blocks, these were just
	       // pauses in between words, so reset silence counter.
	       silenceCount  = 0;
               if(!soundDetected) {
		  // This is the start of a phrase, capture read pointer and start looking for end of the phrase
	     	  phraseStart = rdPtr;
   	     	  soundDetected = 1;
	       }
	    }
	    else if (soundDetected) {
	       // This block does not contain valid audio. This may be just a pause in the sentence, or the end of a sentence.
	       if(silenceCount == 0) {
	          // This is the first empty block after one that contains audio, this may be the end of a phrase
		  phraseEnd = rdPtr + 2*block_frames;
		  if(phraseEnd>=samples) {
	             // Wrap circular buffer
		     phraseEnd = 0;
	          }
 	       }
	       // Count how many empty (silent) blocks we get in a row.
	       silenceCount = silenceCount + 1;
	    }
//            printf("doing analysis sd %d sc %d avg %d acc %d rptr %d\n", soundDetected, silenceCount, avg, acc, rdPtr);
	    // Move on to next block
	    rdPtr = rdPtr + 2*block_frames;
	    if (rdPtr>=samples) {
	       // Wrap circular buffer
	       rdPtr = 0;
	    }
 	    flagIdx = flagIdx + 1;
 	    if(flagIdx >= blocks) {
	       // Wrap circular buffer
 	       flagIdx = 0;
	       // 2-phase handshake: we toggle polarity of the flag once we wrap
	       if(flagPolarity == 0) {
	          flagPolarity = 1;
	       }
	       else {
	          flagPolarity = 0;
	       }
 	    }
	    if (silenceCount >= MIN_SILENCE/cont_cap_block_length) {
	       // The number of empty blocks exceeds the limit: we assume the sentence has ended so we're done
	       // Note CCSR may misinterpret long mid-sentence silences as sentence ends, may need to fine-tune
	       if (phraseEnd>phraseStart){
		  phraseSize = (phraseEnd - phraseStart)/2;  // Number of frames of the captures sentence
	       }
	       else{
                  // The captured sentence straddles the circular buffer loop-point
		  phraseSize = ((samples - phraseStart) + phraseEnd)/2;
	       }
	       printf("Captured phrase b %d e %d sz %d\n", phraseStart, phraseEnd, phraseSize);
	       // Break out of continuous VR loop, go write out this audio to a file for speech2text and NLP
	       break;
	       }
	 }
         printf("stopping continous VR\n");	
	 ccsrState.continuousVoiceRecognitionOn = 0; // Stop continuous VR. Depending on the NLP response, we will turn it back on explicitely
         pthread_join(threadSoundCapture, NULL);     // wait for audio capture to terminate
         snd_pcm_close (capture_handle);             // close audio
	 pthread_mutex_unlock(&semAudio);            // release audio device
	 powerMicAmp(OFF);                           // Turn mic-amp off
         // Signal we captured a phrase
	 write(pipeSoundGen[IN], &sound[singleA], sizeof(sound[singleA]));
	 // Allocate audio buffer for captured phrase for sound-file generation
	 capture_buffer_mono = (short*) malloc((phraseSize/WAV_DOWNSAMPLE_FACTOR) *sizeof(short));
 	 p=0;
	 // Cut captured phrase from circular buffer, downsample and make mono, paste in new buffer
	 if (phraseEnd>phraseStart){
	    for(q=phraseStart;q<phraseEnd;q=q+2*WAV_DOWNSAMPLE_FACTOR){
	       capture_buffer_mono[p] = cont_capture_buffer[q];
	       p=p+1;
	    }
	 }
	 else {
            // The captured sentence straddles the circular buffer loop-point, so cut in 2-stages
	    for(q=phraseStart;q<samples;q=q+2*WAV_DOWNSAMPLE_FACTOR){
	       capture_buffer_mono[p] = cont_capture_buffer[q];
	       p=p+1;
	    }
	    for(q=0;q<phraseEnd;q=q+2*WAV_DOWNSAMPLE_FACTOR){
	       capture_buffer_mono[p] = cont_capture_buffer[q];
	       p=p+1;
	    }
	 }
	 // Write audio file to disk: GoogleVoiceToText.sh deamon is
	 // waiting for this file to appear, and will post it to
	 // Google voice to text API. The returned json file will be
	 // parsed by CCSR_NLP.py, which will call ccsr back through
	 // a telemetry channel. GoogleVoiceToText.sh deamon will delete
	 // voice file after use, and resum polling
	 writeSndFile((phraseSize/WAV_DOWNSAMPLE_FACTOR), capture_buffer_mono);
         // free allocated buffers
	 free(capture_buffer_mono);
	 free(cont_capture_buffer);
	 free(captureBufferFlags);
         // Update emotions. Talking to him makes him a little happy and excited. This may wake CCSR if he was asleep
         setMood(LOW_HAPPY_INC, LOW_AROUSAL_INC);
      }
   }
}



void *soundGenerator() {
   soundType sound;
   int     result, i;
   logMsg(logFile, "Initializing sound generator", LOG);	   
   while(1){
      result = read (pipeSoundGen[OUT],&sound,sizeof(sound));
      if (result != 1) {
         logMsg(logFile, "Unsuccessful read from pipeSoundGen", ERROR);	   
      }
      pthread_mutex_lock(&semAudio);
      initAudio(&playback_handle, SND_PCM_STREAM_PLAYBACK);
      for (i=0; i<sound.notes; i++){
     	      playWave(sound.freq[i], sound.length[i], sound.vol[i], sound.style);
      }
      snd_pcm_drain (playback_handle);
      snd_pcm_close (playback_handle);
      pthread_mutex_unlock(&semAudio);
   }
}
