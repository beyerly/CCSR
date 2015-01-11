

typedef struct expressionType {
   int type;
   int length;
} expressionType;

enum expressionset       {EXPR_BLINK,
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

void facialInit();

