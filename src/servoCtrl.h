


#define PCA9685_ADDR 0x41
// assuming 640x480 camera pic
#define X_MID_IMAGE 320
#define Y_MID_IMAGE 240

#ifdef __cplusplus
extern "C" {
#endif
 
void configServoControl();
void setPanTilt(int pan, int tilt, int speed);
void setArm(int arm, int elbow, int wrist, int hand, int speed);
int pantiltDiagnostics();
void *camtrack();

#ifdef __cplusplus
}
#endif

#define MODE0_SLEEP_offset 4
#define MODE1_AI_offset 5

#define MODE0_SLEEP 1
#define MODE0_NORMAL 0
#define MODE1_AI 1

#define PCA9685_REG_MODE0 0x0
#define PCA9685_REG_MODE1 0x1
// Pan servo
#define PCA9685_REG_LED0_ON_L 0x6
#define PCA9685_REG_LED0_ON_H 0x7
#define PCA9685_REG_LED0_OFF_L 0x8
#define PCA9685_REG_LED0_OFF_H 0x9
// Tilt Servo
#define PCA9685_REG_LED1_ON_L 0xA
#define PCA9685_REG_LED1_ON_H 0xB
#define PCA9685_REG_LED1_OFF_L 0xC
#define PCA9685_REG_LED1_OFF_H 0xD
// Arm servo
#define PCA9685_REG_LED2_ON_L 0xE
#define PCA9685_REG_LED2_ON_H 0xF
#define PCA9685_REG_LED2_OFF_L 0x10
#define PCA9685_REG_LED2_OFF_H 0x11
// elbow servo
#define PCA9685_REG_LED3_ON_L 0x12
#define PCA9685_REG_LED3_ON_H 0x13
#define PCA9685_REG_LED3_OFF_L 0x14
#define PCA9685_REG_LED3_OFF_H 0x15
// Wrist servo
#define PCA9685_REG_LED4_ON_L 0x16
#define PCA9685_REG_LED4_ON_H 0x17
#define PCA9685_REG_LED4_OFF_L 0x18
#define PCA9685_REG_LED4_OFF_H 0x19
// Hand servo
#define PCA9685_REG_LED5_ON_L 0xA
#define PCA9685_REG_LED5_ON_H 0xB
#define PCA9685_REG_LED5_OFF_L 0xC
#define PCA9685_REG_LED5_OFF_H 0xD



#define PCA9685_REG_PRESCALE 0xFE

#define PAN_SERVO_MIN  147 // this is the 'minimum' pulse length count (out of 4096)
#define PAN_SERVO_MAX  650 // this is the 'maximum' pulse length count (out of 4096)
#define TILT_SERVO_MIN  312 // this is the 'minimum' pulse length count (out of 4096)
#define TILT_SERVO_MAX  515 // this is the 'maximum' pulse length count (out of 4096)

#define ARM_SERVO_MIN  200 // this is the 'minimum' pulse length count (out of 4096)
#define ARM_SERVO_MAX  373 // this is the 'maximum' pulse length count (out of 4096)
#define ELBOW_SERVO_MIN  132 // this is the 'minimum' pulse length count (out of 4096)
#define ELBOW_SERVO_MAX  718 // this is the 'maximum' pulse length count (out of 4096)
#define WRIST_SERVO_MIN  155 // this is the 'minimum' pulse length count (out of 4096)
#define WRIST_SERVO_MAX  661 // this is the 'maximum' pulse length count (out of 4096)
#define HAND_SERVO_MIN  234 // this is the 'minimum' pulse length count (out of 4096)
#define HAND_SERVO_MAX  526 // this is the 'maximum' pulse length count (out of 4096)

#define PAN_SERVO_NEUTRAL 397
#define TILT_SERVO_NEUTRAL 425

#define PW_STEP 1

#define PAN_ANGLE_RANGE 180 // degrees
#define TILT_ANGLE_RANGE 90 // degrees

#define ARM_ANGLE_RANGE 45 // degrees
#define ELBOW_ANGLE_RANGE 180 // degrees
#define WRIST_ANGLE_RANGE 180 // degrees
#define HAND_ANGLE_RANGE 180 // degrees


// The CAMERA_TRACK_INTERVAL is the interval the camera tracking process uses to move the camera
// to center the tracked object. Since the opencv functions in visual.cpp are slow, setting this 
// value too low (fast) will cause the camera to overshoot. Setting it too high will cause trackin gto 
// be too slow to be usefull. With the current opencv processign speed, the value below is workable,
// but we need to speed the visual processing up, and then lower thsi interval to be more smooth
#define CAMERA_TRACK_INTERVAL 350000  // ms
