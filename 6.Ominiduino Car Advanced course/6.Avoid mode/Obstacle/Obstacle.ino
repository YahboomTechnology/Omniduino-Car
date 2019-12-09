/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Obstacle.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Avoid mode
* @details
* @par History  
*/
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include "Wire.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define IR_SENSOR_L1 A3
#define IR_SENSOR_L2 A0
#define IR_SENSOR_R1 A2
#define IR_SENSOR_R2 A1
#define IR_SENSOR_MID A7

#define BUZZER 10 //Define buzzer pins
#define KEY_PIN 8 //Define K1 button pins
#define RGB_PIN 9 //Define RGB pins
#define MAX_LED 4 //Car with 4 RGB lights
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_LED, RGB_PIN, NEO_RGB + NEO_KHZ800);

/*car speed control*/
int CarSpeedControl = 60;

//button status
bool button_press = false;

/*Define variables to save the size of the data collected by the infrared sensor*/
int ir_L1;  //left front
int ir_L2;  //left rear
int ir_R1;  //right front
int ir_R2;  //right rear
int ir_Mid; //Middle front

/*Define variables to save whether the infrared sensor detects an obstacle*/
int dig_ir_l1; //left front 
int dig_ir_l2; //left rear
int dig_ir_r1; //right front
int dig_ir_r2; //right rear

/**
* Function       run
* @author        chengengyue
* @date          2019.10.08
* @brief         advance
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void run(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed);    //Right front wheel Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);     //Right rear wheel Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed);    //Left front wheel Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed);    //Left rear wheel Forward
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
  pwm.setPWM(11, 0, Speed);    //Right front wheel Reverse
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);     //Right  rear wheel Reverse

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed);    //Left front wheel Reverse
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);    //Left rear wheel Reverse
}

/**
* Function       left
* @author        chengengyue
* @date          2019.10.08
* @brief         car left pan(A type wheel Reverse，B type wheel Forward)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void left(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed);   //Right front wheel(B type) Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);    //Right rear wheel(A type) Reverse

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed);   //Left front wheel (A type) Reverse
  pwm.setPWM(15, 0, Speed);   //Left rear wheel(B type) Forward
  pwm.setPWM(14, 0, 0);
}

/**
* Function       right
* @author        chengengyue
* @date          2019.10.08
* @brief         car right pan(A type wheel Forward，B type wheel Reverse)
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
* @brief         car spin right(Left wheel Forward,Right wheel Reverse)
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
* Function       clearRGB
* @author        chengengyue
* @date          2019.10.08
* @brief         Turn off onboard RGB light
* @param[in1]    void
* @param[out]    void
* @retval        void
* @par History   no
*/
void clearRGB()
{
  uint32_t color = strip.Color(0, 0, 0);
  for (uint8_t i = 0; i < MAX_LED; i++)
  {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

/**
* Function       showRGB
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in1]    num  Select the serial number of the RGB. 
*                     If it is greater than or equal to the maximum number of the RGB, it will be fully illuminated.
* @param[in2]    R    R value(0~255)
* @param[in3]    G    G value(0~255)
* @param[in4]    B    B value(0~255)
* @param[out]    void
* @retval        void
* @par History   no
*/
void showRGB(int num, int R, int G, int B)
{
  uint32_t color = strip.Color(G, R, B);
  if (num > MAX_LED - 1)  //All RGB be illuminated
  {
    for (int i = 0; i < MAX_LED; i++)
    {
      strip.setPixelColor(i, color);
    }
  }
  else                   //Lighting a RGB separately
  {
    strip.setPixelColor(num, color);
  }
  strip.show();
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
        whistle();
        return;
      }
    }
  }
}

/*
* Function       whistle
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void whistle()
{
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(BUZZER, HIGH); //sound
    delay(3);                   
    digitalWrite(BUZZER, LOW);  //no sound
    delay(1);                   
  }
}




/**
* Function       ir_rgb
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void ir_rgb()
{
  //Read IR sensor if detect obstacle
	dig_ir_l1 = digitalRead(IR_SENSOR_L1);
	dig_ir_l2 = digitalRead(IR_SENSOR_L2);
	dig_ir_r1 = digitalRead(IR_SENSOR_R1);
	dig_ir_r2 = digitalRead(IR_SENSOR_R2);

  //If sensor detected the obstacle,the RGB light next to it lights up red.
	if (dig_ir_l1 == LOW)
		showRGB(3, 255, 0, 0);
	else
		showRGB(3, 0, 0, 0);
    
	if (dig_ir_l2 == LOW)
		showRGB(0, 255, 0, 0);
	else
		showRGB(0, 0, 0, 0);
    
	if (dig_ir_r1 == LOW)
		showRGB(2, 255, 0, 0);
	else
		showRGB(2, 0, 0, 0);
    
	if (dig_ir_r2 == LOW)
		showRGB(1, 255, 0, 0);
	else
		showRGB(1, 0, 0, 0);
}

/**
* Function       obstacle
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   
*/
void obstacle()
{
  //Read IR sensor data
	dig_ir_l1 = digitalRead(IR_SENSOR_L1);
	dig_ir_l2 = digitalRead(IR_SENSOR_L2);
	dig_ir_r1 = digitalRead(IR_SENSOR_R1);
	dig_ir_r2 = digitalRead(IR_SENSOR_R2);
	ir_Mid = analogRead(IR_SENSOR_MID) / 10;
	ir_L1 = analogRead(IR_SENSOR_L1) / 10;
	ir_L2 = analogRead(IR_SENSOR_L2) / 10;
	ir_R1 = analogRead(IR_SENSOR_R1) / 10;
	ir_R2 = analogRead(IR_SENSOR_R2) / 10;

  //According to the situation detected by the sensor, the corresponding obstacle avoidance movement is made.
	if (ir_Mid < 80 && ir_L1 < 30 && ir_R1 < 30 && ir_R2 < 30)  
	{
		turn_direction(-1);
	}
	else if (ir_Mid < 80 && ir_L1 < 30 && ir_R1 < 30 && ir_L2 < 30)
	{
		turn_direction(1);
	}
	else if (ir_Mid < 80 && ir_L1 < 40 && ir_R1 < 40)  
	{
		back(CarSpeedControl);
		delay(100);
		turn_direction(2);
	}
	else if (ir_L1 < 40 && ir_R1 < 40)  
	{
		run(CarSpeedControl);
		delay(100);
	}
	else if (ir_L1 < 40 && ir_L2 < 40)  
	{
		right(CarSpeedControl);
	}
	else if (ir_Mid < 80 && ir_L1 < 40 && ir_L2 < 40)  
	{
		right(CarSpeedControl);
	}
	else if (ir_R1 < 40 && ir_R2 < 40)  
	{
		left(CarSpeedControl);
	}
	else if (ir_Mid < 80 && ir_R1 < 40 && ir_R2 < 40)  
	{
		left(CarSpeedControl);
	}
	else if (ir_Mid < 80 && dig_ir_l1 == LOW && dig_ir_r1 == HIGH) 
	{
		spin_right(CarSpeedControl);
	}
	else if (ir_Mid < 80 && dig_ir_l1 == HIGH && dig_ir_r1 == LOW) 
	{
		spin_left(CarSpeedControl);
	}
	else if (dig_ir_l1 == HIGH && dig_ir_r1 == LOW)    
	{
		spin_left(CarSpeedControl);
	}
	else if (dig_ir_l1 == LOW && dig_ir_r1 == HIGH)  
	{
		spin_right(CarSpeedControl);
	}
	else if (ir_Mid < 90)  
	{
		back(CarSpeedControl);
		delay(100);
		turn_direction(2);
	}
	else   //In other cases, the car keeps going straight.
	{
		run(CarSpeedControl);
	}
}

/**
* Function       turn_direction
* @author        chengengyue
* @date          2019.10.08
* @brief         car turn_direction
* @param[in]     degree
* @param[out]    void
* @retval        void
* @par History   no
*/
void turn_direction(int degree)
{
	switch (degree)
	{
	case 1:
		spin_right(65);
		delay(200);
		brake();
		delay(1);
		break;
	case 2: 
		spin_right(65);
		delay(380);
		brake();
		delay(1);
		break;
	case -1:
		spin_left(65);
		delay(200);
		brake();
		delay(1);
		break;
	case -2: 
		spin_left(65);
		delay(380);
		brake();
		delay(1);
		break;
	
	default:
		break;
	}
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
void setup()
{
  pinMode(KEY_PIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(IR_SENSOR_L1, INPUT);
  pinMode(IR_SENSOR_L2, INPUT);
  pinMode(IR_SENSOR_R1, INPUT);
  pinMode(IR_SENSOR_R2, INPUT);
  pinMode(IR_SENSOR_MID, INPUT);

  //Enable I2C communication
  Wire.begin();
    
  strip.begin();
  strip.show();
  clearRGB();                    
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
* @param[out]    void
* @retval        void
* @par History   no
*/
void loop()
{
  keyscan();
  if (button_press)
  {
    ir_rgb();   
    obstacle(); // start avoid mode
  }
  else
  {
    brake();    
    clearRGB(); 
  }
}
