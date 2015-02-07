#define MOOD_PERIOD 1000000 // 1 sec
#define MIN_HAPPINESS -255
#define MAX_HAPPINESS 255
#define MIN_AROUSAL 0
#define MAX_AROUSAL 255
#define MOOD_DEGRADATION_SPEED 10
#define MED_HAPPY_INC 50
#define MED_AROUSAL_INC 50
#define LOW_HAPPY_INC 10
#define LOW_AROUSAL_INC 10
#define HIGH_HAPPY_INC 100
#define HIGH_AROUSAL_INC 100;
#define MOOD_NORMAL 0


void *mood();
void moodInit();
void setMood(int h, int a);
void HVtoRGB( float *r, float *g, float *b, float h, float v );