#ifndef _PCA_ACTION_H
#define _PCA_ACTION_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SAFE_T  50

#define MG90_A(x) ( (uint16_t)(102.4 + 2.275*x) )
#define MG90_B(x) ( (uint16_t)(102.4 + 2.275*(180-x)) )

#define RF_A  3
#define RF_B  2
#define LF_A  11
#define LF_B  10
#define RB_A  1
#define RB_B  0
#define LB_A  9
#define LB_B  8

void init_pac9685(void);

void quad_forward(void);
void quad_backward(void);
void quad_left_rotation(void);
void quad_right_rotation(void);
uint8_t quad_ptz_up(void);
uint8_t quad_ptz_down(void);

void quad_reset_all(void);

void setServoSerial( uint16_t* pwms_off);
void setServoSerialRLAction( uint16_t* left_pwms_off, uint16_t* rigth_pwms_off);
void setServoSerialPTZAction( uint16_t* roll_pitch_pwms_off);

#endif
