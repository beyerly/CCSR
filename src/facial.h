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
   EXPR_BLINK,
   EXPR_TALK,
   EXPR_LOOKSTRAIGHT,
   EXPR_LOOKLEFT,
   EXPR_LOOKRIGHT,
   EXPR_LOOKUP,
   EXPR_LOOKDOWN,
   EXPR_SLEEP,
   EXPR_HAPPY,
   EXPR_WAKE,
   EXPR_ANGRY,
   EXPR_SCARED,
   EXPR_CROSSEYED,
   EXPR_SCANNER,   // unused, no scannel HW yet
   NUM_EXPR
};

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


