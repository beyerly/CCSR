void *facialExpressions();
void dispEnable(int i2cAddr);
void facialInit();
void dispSetBrightness(int i2cAddr, char b);
void dispSetBlinkRate(int i2cAddr, int b);
void writeDisplay(int i2cAddr, char* dispBuff);
void draw_bmp_8x8(char* dispBuf, char* bitmap, char* mask);
void draw_bmp_16x8(char* dispBuf, char* bitmap, char* mask);


typedef struct expressionType {
   int type;
   int length;
} expressionType;

enum expressionset       {
   EXPR_BLINK,              // 0
   EXPR_TALK,               // 1
   EXPR_LOOKSTRAIGHT,       // 2
   EXPR_LOOKLEFT,           // 3
   EXPR_LOOKRIGHT,          // 4
   EXPR_LOOKUP,             // 5
   EXPR_LOOKDOWN,           // 6
   EXPR_SLEEP,              // 7
   EXPR_HAPPY,              // 8
   EXPR_WAKE,               // 9
   EXPR_ANGRY,              // 10
   EXPR_SCARED,             // 11
   EXPR_CROSSEYED,          // 12
   EXPR_SCANNER,            // 13 unused, no scanner HW yet
   EXPR_NODYES,             // 14
   EXPR_SHAKENO,            // 15
   EXPR_WHITELIGHT,         // 16
   NUM_EXPR
} expressionSetType;

#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0
#define HT16K33_BLINK_2HZ  1
#define HT16K33_BLINK_1HZ  2
#define HT16K33_BLINK_HALFHZ  3

#define HT16K33_CMD_BRIGHTNESS 0xE0

#define EYE_R_ADDR 0x70
#define EYE_L_ADDR 0x71
#define MOUTH_ADDR 0x72
#define DISP_ROWS 16       // 8x16 displays
#define DISP_PAGES 2


#define EXPR_BLINK_FRAME_RATE 25000


