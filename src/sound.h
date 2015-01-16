
#define DEV_AUDIO "/dev/snd/pcmC0D0p"
#define VOICE_FILE "/home/root/ccsr/data/voice.wav" 
#define VOICE_TO_TXT_FILE "text.txt"
#define MAX_TXT_SIZE 60
#define BUF_SIZE 32000
#define WAV_BUF_SIZE 1024 
#define PLAYBACK_SPEED 44100
#define WAV_DOWNSAMPLE_FACTOR 4   // So we are downsampling to 11025
#define ENERGY_DOWNSAMPLE_FACTOR 16   // So we are downsampling to 11025

#define NUM_SOUNDS 6
#define MAX_NOTES 8
#define AVG_WINDOW_SIZE 16
#define NOISE_LEVEL 1000
#define AUDIOCAPTURE_TIMEOUT 3 // sec

#define SND_REPETITION_DELAY 10000

#define AUDIO_PEAK_HYSTRESIS 10
#define AUDIO_ENERGY_HYSTRESIS 10
#define EAR_PERIOD 500000          // 1s

#define MIN_SILENCE 2000 //2s

#define MAX_FADE_VOL 1000
#define FADE_TIME 1000


#define NOTE_A4  300 
#define NOTE_Cs5 400
#define NOTE_E5  500
#define NOTE_F5  600
#define NOTE_A5  700
#define NOTE_C6  800
#define NOTE_F6  900
#define NOTE_G6  1000
#define NOTE_G7  1100
#define NOTE_G8  1200

enum soundStyle {sine, square, sineDetune, squareFade, sineFade};
enum listenMode {DETECT_NOISE_LEVEL, LISTEN, PLAYBACK, CAPTURE, VOICE_RECOGNITION};
enum standardSounds {keyPressed,
                     linuxBooted,
		     terminate,
                     batteryAlarm,
                     currentAlarm, 
                     singleA, 
		     doubleA, 
		     observeSnd, 
		     orientationSnd, 
		     evasiveActionSnd,
		     standardSoundsCount};

typedef struct soundType {
   int freq[MAX_NOTES];
   int length[MAX_NOTES];
   int vol[MAX_NOTES];
   int style;
   int notes;
} soundType;


void initSounds();
void initEspeak();
int initAudio();
void set_volume(long volume);
int playWave(int freq, int length, int vol, int style);
void captureAudio(int* peak, long* energy, short* buffer, int samples);
void recordWave(int mode, int length);
int playAudioMemory(int sample);
void genSine(short* buffer, int size);
void genSquare(short* buffer, int size);
void *soundGenerator();
void *ears();
void say(char *text);
void drainAudioPlayback(); 
