/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         CarRun01.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        
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
* @param[in]     Speed(0-160)
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
* Function       back
* @author        chengengyue
* @date          2019.10.08
* @brief         car back
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void back(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, Speed);    //Right front reserve
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);     //Right rear reserve

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed);    //Left front reserve
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);    //Left front reserve
}

/**
* Function       left
* @author        chengengyue
* @date          2019.10.08
* @brief         car left translation(A type wheel Reverse，B type wheel Forward)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void left(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed);   //Right rear wheel(B type) Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);    //Right front wheel(A type) Reverse

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed);   //Left rear wheel (A type) Reverse
  pwm.setPWM(15, 0, Speed);   //Left front wheel(B type) Forward
  pwm.setPWM(14, 0, 0);
}

/**
* Function       right
* @author        chengengyue
* @date          2019.10.08
* @brief         car right translation(A type wheel Forward，B type wheel Reverse)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void right(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, Speed);   //Right front wheel(B type) Reverse
  pwm.setPWM(8, 0, Speed);    //Right rear wheel(A type) Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed);   //Left front wheel(A type) Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);   //Left rear wheel(B type) Reverse
}

/**
* Function       spin_left
* @author        chengengyue
* @date          2019.10.08
* @brief         car spin left(Left wheel Reverse,Right wheel Forward)
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   no
*/
void spin_left(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed);   //Right front wheel Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);    //Right rear wheel Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed);   //Left front wheel  Reserve
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);   //Left rear wheel  Reserve
}

/**
* Function       spin_right
* @author        chengengyue
* @date          2019.06.25
* @brief         car spin right(Left wheel Forward,Right wheel Reserve)
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   no
*/
void spin_right(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, Speed);   //Right front wheel Reserve
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);    //Right rear wheel Reserve

  pwm.setPWM(13, 0, Speed);   //Left front wheel Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed);   //Left rear wheel Forward
  pwm.setPWM(14, 0, 0);
}


/**
* Function       carRun01
* @author        chengengyue
* @date          2019.10.08
* @brief         carRun01
* @param[in]     void
* @retval        void
* @par History   no
*/
void carRun01()
{
  right(CarSpeedControl);         
  delay(1000);                          
  back(CarSpeedControl);            
  delay(1000);                          
  left(CarSpeedControl);          
  delay(1000);                          
  run(CarSpeedControl);             
  delay(1000);    
  brake();                         
  delay(500);
  spin_left(CarSpeedControl);       
  delay(2000);
  brake();                         
  delay(500);
  spin_right(CarSpeedControl);     
  delay(2000);
  brake();                          
}

/**
* Function       keyscan
* @author        chengengyue
* @date          2019.10.08
* @brief         key scan
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
* @brief         Initialization settings
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
  //keyscan
  keyscan();

  if (button_press)     //when button is pressed
  {
    delay(500);         
    carRun01();         
    button_press = 0;   
  }
}
