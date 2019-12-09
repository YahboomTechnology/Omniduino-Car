/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         CarRun.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Button control car advance
* @details
* @par History  no
*/
//Import library file
#include <Adafruit_PWMServoDriver.h>
#include "Wire.h"

#define KEY_PIN 8    //Define key pin

//PCA9685 Initialization
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//Car control parameters
int CarSpeedControl = 60;

//Button status
bool button_press = false;

/**
* Function       run
* @author        chengengyue
* @date          2019.10.08
* @brief         Advance
* @param[in]     Speed(40-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void run(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed);    //Right rear wheel Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);     //Right front wheel Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed);    //Left rear wheel Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed);    //Left front wheel Forward
  pwm.setPWM(14, 0, 0);
}

/**
* Function       brake
* @author        chengengyue
* @date          2019.10.08
* @brief         car stop
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void brake()
{
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(10, 0, 0);

  pwm.setPWM(12, 0, 0);
  pwm.setPWM(13, 0, 0);
  pwm.setPWM(14, 0, 0);
  pwm.setPWM(15, 0, 0);
}

/**
* Function       keyscan
* @author        chengengyue
* @date          2019.10.08
* @brief         keyscan
* @param[in1]    void
* @retval        void
* @par History   no
*/
void keyscan()
{
  int val;
  val = digitalRead(KEY_PIN); //Read the digital 8-port level value assigned to val
  if (val == LOW)             //When the button is pressed
  {
    delay(10);                  //Delayed debounce
    val = digitalRead(KEY_PIN); //Read button status again
    while (val == LOW)
    {
      val = digitalRead(KEY_PIN); //Third read button status
      if (val == HIGH)            //Determine if the button is released, release it to execute
      {
        button_press = !button_press;
        return;
      }
    }
  }
}

/**
* Function       setup
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     void
* @retval        void
* @par History   no
*/
void setup() {
  // put your setup code here, to run once:
  pinMode(KEY_PIN, INPUT_PULLUP);
  Wire.begin();       
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  brake();            
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
  //key scan
  keyscan();

  if (button_press)   
  {
    run(CarSpeedControl);
  }
  else               
  {
    brake();
  }
}
