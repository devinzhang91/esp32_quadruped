#include "pca_action.h"
#include "pid.h"
#include "gyroscope.h"

#include <Ticker.h>

Ticker tickerAction;
void timer_action_func();
Ticker tickerPTZ;
void timer_ptz_func();

#define MAX_PTZ_LEVEL       (4)
#define PTZ_LEVEL_ANGLE     (60/MAX_PTZ_LEVEL)
static uint8_t ptz_lv = MAX_PTZ_LEVEL/2;
///record set(wanne) arr
static uint16_t set_arr[16];
///record last(now) arr
static uint16_t last_arr[16];
pid pidAction[16];

// called this way, it uses the default address 0x40
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
Adafruit_PWMServoDriver pwm;

void init_pac9685(void) {
  Serial.println("Init pac9685");
  pwm = Adafruit_PWMServoDriver();

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
  quad_reset_all();
  memcpy(last_arr, set_arr, 16*sizeof(uint16_t));
  
  tickerAction.attach_ms(SAFE_T, timer_action_func);   ///every SAFE_T ms
  tickerPTZ.attach_ms(SAFE_T*2, timer_ptz_func);   ///every SAFE_T ms
}

void timer_action_func(void){
    // for (size_t i = 0; i < 16; i++) {
    //     last_arr[i] += pidAction[i].run(set_arr[i], last_arr[i]);
    // }
    ///L action
    for (size_t i = 0; i < 4; i++) {
        last_arr[i] += pidAction[i].run(set_arr[i], last_arr[i]);
    }
    ///ptz action
    for (size_t i = 4; i < 6; i++) {
        last_arr[i] += pidAction[i].run(set_arr[i], last_arr[i]);
    }
    ///skip 6~11 channel
    ///R action
    for (size_t i = 12; i < 16; i++) {
        last_arr[i] += pidAction[i].run(set_arr[i], last_arr[i]);
    }

    Wire.beginTransmission(PCA9685_I2C_ADDRESS);
    Wire.write(PCA9685_LED0_ON_L);
    for (size_t i = 0; i < 16; i++) {
        Wire.write(0);
        Wire.write(0);
        Wire.write((last_arr[i]) & 0xFF);
        Wire.write((last_arr[i]) >> 8);
    }
    Wire.endTransmission();
}

void timer_ptz_func(void){
    float ypr[3];
    ///yaw/pitch/roll
    // if(0==getYPRDMP(ypr)){
    if(0==getYPRKalman(ypr)){
        // int16_t roll_angle = 80-(int16_t)(ypr[2]* 180/M_PI);
        // int16_t pitch_angle = 90-(ptz_lv-MAX_PTZ_LEVEL/2)*PTZ_LEVEL_ANGLE + (int16_t)(ypr[1]* 180/M_PI);
        int16_t roll_angle = 80-ypr[2];
        int16_t pitch_angle = 90-(ptz_lv-MAX_PTZ_LEVEL/2)*PTZ_LEVEL_ANGLE + ypr[1];

        uint16_t rp_arr[2];
        rp_arr[0] = MG90_A(roll_angle);
        rp_arr[1] = MG90_A(pitch_angle);
        
        Serial.printf("roll_angle=%d, pitch_angle=%d \n",roll_angle, pitch_angle);
        setServoSerialPTZAction(rp_arr);
    }
}

void setServoSerial( uint16_t* pwms_off) {
    memcpy(set_arr, pwms_off, 16*sizeof(uint16_t));
}

///
void setServoSerialRLAction( uint16_t* left_pwms_off, uint16_t* rigth_pwms_off) {
    ///LB_1/LB_2/LF_1/LF_2  channel 0~3
    memcpy(set_arr,     left_pwms_off,  4*sizeof(uint16_t));
    ///RF_1/RF_2/RB_1/RB_2  channel 12~15
    memcpy(set_arr+12,  rigth_pwms_off, 4*sizeof(uint16_t));
}

void setServoSerialPTZAction( uint16_t* roll_pitch_pwms_off) {
    ///roll/pitch  channel 4~5
    memcpy(set_arr+4, roll_pitch_pwms_off, 2*sizeof(uint16_t));
}

enum mg90 {
    LB_1 = 0,
    LB_2 = 1,
    LF_1 = 2,
    LF_2 = 3,

    RF_1 = 0,
    RF_2 = 1,
    RB_1 = 2,
    RB_2 = 3
};
/**
 *  l_arr[] =   (LB_1)  (LB_2)
 *              (LF_1)  (LF_2)
 *  r_arr[] =   (RF_1)  (RF_2)
 *              (RB_1)  (RB_2)
 *
 * 
 *      Left                     Right    
 * ---------------------------------------------
 *                                      |     
 *      |----|[LF_2]--00--[RF_2]|----|  |    Forward
 *      [LF_1]     |      |     [RF_1]  |
 *      \____/     | Body |     \____/  |
 *                 |      |             |
 *                 |      |             |
 *      |----|[LB_2]------[RB_2]|----|  |    Back
 *      [LB_1]                  [RB_1]  |
 *      \____/                  \____/  |
 * 
 * ---------------------------------------------
 * 
 *      --------------|
 *      [LB_2] |      |     
 *      -------|[LB_1]|    angle(LB_1)=90
 *             |      |
 *              \____/
 * 
 * 
 *      --------------\
 *      [LB_2]|        \    
 *      -------\ [LB_1] \   angle(LB_1)>90
 *              \        \
 *               \_______/
 * 
 * ---------------------------------------------
 * 
 *    ----|[LF_2]|--00--|[RF_2]|----
 *               | Body |  
 *        |[LB_2]|------|[RB_2]|     
 *                                  angle(LF_2)=90
 *                                  angle(RF_2)=90
 * 
 * 
 *      \------\         /-------/
 *       \      \       /       /
 *        |[LF_2]|--00--|[RF_2]|
 *               | Body |  
 *        |[LB_2]|------|[RB_2]|     
 *                                  angle(LF_2)>90
 *                                  angle(RF_2)>90
 * 
*/          

void quad_forward(void){
  static uint8_t f = 0;
  if(++f%2){
    uint16_t l_arr[] = {MG90_A(90),  MG90_A(120), 
                        MG90_B(120), MG90_B(120)};

    uint16_t r_arr[] = {MG90_A(90),  MG90_A(90), 
                        MG90_B(90),  MG90_A(90) };
    setServoSerialRLAction(l_arr, r_arr);
    delay(3*SAFE_T);
    l_arr[LF_1] = MG90_B(90);
    setServoSerialRLAction(l_arr, r_arr);
  } else {
    uint16_t l_arr[] = {MG90_A(90),  MG90_A(90), 
                        MG90_B(90),  MG90_B(90)};
    uint16_t r_arr[] = {MG90_A(120), MG90_A(120), 
                        MG90_B(90),  MG90_B(120) };
    setServoSerialRLAction(l_arr, r_arr);
    delay(3*SAFE_T);
    r_arr[RF_1] = MG90_A(90);
    setServoSerialRLAction(l_arr, r_arr);
  }
    delay(SAFE_T);
}


void quad_backward(void){
  static uint8_t f = 0;
  if(++f%2){
    uint16_t l_arr[] = {MG90_A(120), MG90_A(120), 
                        MG90_B(90),  MG90_B(120)}; 
    uint16_t r_arr[] = {MG90_A(90),  MG90_A(90), 
                        MG90_B(90),  MG90_B(90) };
    setServoSerialRLAction(l_arr, r_arr);
    delay(3*SAFE_T);
    l_arr[LB_1] = MG90_A(90);
    setServoSerialRLAction(l_arr, r_arr);
  } else {
    uint16_t l_arr[] = {MG90_A(90),  MG90_A(90), 
                        MG90_B(90),  MG90_B(90)};
    uint16_t r_arr[] = {MG90_A(90),  MG90_A(120), 
                        MG90_B(120), MG90_B(120)};
    setServoSerialRLAction(l_arr, r_arr);
    delay(3*SAFE_T);
    r_arr[RB_1] = MG90_B(90);
    setServoSerialRLAction(l_arr, r_arr);
  }
    delay(SAFE_T);
}

void quad_left_rotation(void){
  static uint8_t f = 0;
  if(++f%2){
    uint16_t l_arr[] = {MG90_A(120),  MG90_A(120), 
                        MG90_B(90),  MG90_B(120)};
    uint16_t r_arr[] = {MG90_A(120),  MG90_A(120), 
                        MG90_B(90),  MG90_B(120) };
    setServoSerialRLAction(l_arr, r_arr);
    delay(3*SAFE_T);
    l_arr[LB_1] = MG90_A(90);
    r_arr[RF_1] = MG90_A(90);
    setServoSerialRLAction(l_arr, r_arr);
  }else{
    uint16_t l_arr[] = {MG90_A(90),  MG90_A(90), 
                        MG90_B(120),  MG90_B(90)};
    uint16_t r_arr[] = {MG90_A(90),  MG90_A(90), 
                        MG90_B(120),  MG90_B(90) };
    setServoSerialRLAction(l_arr, r_arr);
    delay(3*SAFE_T);
    l_arr[LF_1] = MG90_B(90);
    r_arr[RB_1] = MG90_B(90);
    setServoSerialRLAction(l_arr, r_arr);
  }
    delay(SAFE_T);
}

void quad_right_rotation(void){
  static uint8_t f = 0;
  if(++f%2){
    uint16_t l_arr[] = {MG90_A(90),  MG90_A(120), 
                        MG90_B(120),  MG90_B(120)}; 
    uint16_t r_arr[] = {MG90_A(90),  MG90_A(120), 
                        MG90_B(120),  MG90_B(120) };
    setServoSerialRLAction(l_arr, r_arr);
    delay(3*SAFE_T);
    l_arr[LF_1] = MG90_B(90);
    r_arr[RB_1] = MG90_B(90);
    setServoSerialRLAction(l_arr, r_arr);
  }else{
    uint16_t l_arr[] = {MG90_A(120),  MG90_A(90), 
                        MG90_B(90),  MG90_B(90)};
    uint16_t r_arr[] = {MG90_A(120),  MG90_A(90), 
                        MG90_B(90),  MG90_B(90) };
    setServoSerialRLAction(l_arr, r_arr);
    delay(3*SAFE_T);
    l_arr[LB_1] = MG90_A(90);
    r_arr[RF_1] = MG90_A(90);
    setServoSerialRLAction(l_arr, r_arr);
  }
    delay(SAFE_T);
}

uint8_t quad_ptz_up(void){
    if(ptz_lv < MAX_PTZ_LEVEL) 
        ptz_lv++;
    return ptz_lv;
}
uint8_t quad_ptz_down(void){
    if(ptz_lv > 0) 
        ptz_lv--;
    return ptz_lv;
}

void quad_reset_all(void){
    uint16_t _arr[] = { MG90_A(90),  MG90_A(90), 
                        MG90_B(90),  MG90_B(90), 
                        MG90_B(90),  MG90_A(90), 
                        MG90_A(90),  MG90_A(90),
                        MG90_A(90),  MG90_A(90), 
                        MG90_A(90),  MG90_A(90), 
                        MG90_A(90),  MG90_A(90), 
                        MG90_B(90),  MG90_B(90) };
    setServoSerial(_arr);
    delay(SAFE_T);
}
