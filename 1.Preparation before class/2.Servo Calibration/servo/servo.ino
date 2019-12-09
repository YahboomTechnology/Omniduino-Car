/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         servo.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Servo Calibration
* @details
* @par History  
*/
//Import
#include "./Adafruit_PWMServoDriver.h"
#include "Wire.h"

//Initialization PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int servo_degree = 90;

/**
* Function       servo
* @author        chengengyue
* @date          2019.10.08
* @brief         control servo angle
* @param[in1]    void
* @retval        void
* @par History   no
*/
void servo(int degree)
{
  long us = (degree * 1800 / 180 + 600); // 0.6 ~ 2.4
  long pwmvalue = us * 4096 / 20000;     // 50hz: 20,000 us
  pwm.setPWM(7, 0, pwmvalue);
}

/**
* Function       setup
* @author        chengengyue
* @date          2019.10.08
* @brief         Initialization setting
* @param[in]     void
* @retval        void
* @par History   no
*/
void setup() {
  // put your setup code here, to run once:
  Wire.begin();       
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  servo(servo_degree);  
}

/**
* Function       loop
* @author        chengengyue
* @date          2019.10.08
* @brief         main function
* @param[in]     void
* @retval        void
* @par History   no
*/
void loop() {
  // put your main code here, to run repeatedly:
}
