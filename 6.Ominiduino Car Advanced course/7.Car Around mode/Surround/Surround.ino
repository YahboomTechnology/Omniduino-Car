/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Surround.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Surround
* @details
* @par History  
*/
//Import library
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include "Wire.h"


#define BUZZER 10
#define KEY_PIN 8		//Define K1 button pins
#define RGB_PIN 9		//Define RGB pins
#define MAX_LED 4		//Car with 4 RGB lights

#define IR_SENSOR_L1 A3
#define IR_SENSOR_L2 A0
#define IR_SENSOR_R1 A2
#define IR_SENSOR_R2 A1
#define IR_SENSOR_MID A7

//Initialize pca9685, RGB programming light
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_LED, RGB_PIN, NEO_RGB + NEO_KHZ800);

/*====================================================================================================
PID Function
The PID function is used in mainly
control applications. PIDCalc performs one iteration of the PID
algorithm.
While the PID function works, main is just a dummy program showing
a typical usage.
=====================================================================================================*/
typedef struct
{
	float SetPoint;   //Desired value
	float Proportion; // Proportional Const
	float Integral;   // Integral Const
	float Derivative; // Derivative Const
	float LastError;  // Error[-1]
	float PrevError;  // Error[-2]
	float SumError;   // Sums of Errors
} PID;

/*====================================================================================================/
PID Calculation part
=====================================================================================================*/
PID PID_MID = {60, 0.5, 0, 0, 0, 0, 0};
PID PID_IR = {40, 1.2, 0, 0, 0, 0, 0};
float PIDCal_IR(PID pid, float nowValue);


/*Define variables to save the size of the data collected by the infrared sensor*/
int ir_L1; //left front
int ir_L2; //left rear
int ir_R1; //right front
int ir_R2; //right rear
int ir_Mid; //Middle front

/*Define variables to save whether the infrared sensor detects an obstacle*/
int dig_ir_l1; //left front 
int dig_ir_l2; //left rear
int dig_ir_r1; //right front
int dig_ir_r2; //right rear

//button status
bool button_press = false;

const char wheel[4][2] = {{10, 11}, {13, 12}, {15, 14}, {8, 9}};
int CarSpeedControl = 60;
int car_direction = 0;   //Set the direction of the car surround mode, 0 is left and 1 is right

/**
* Function       run
* @author        chengengyue
* @date          2019.10.08
* @brief         advance
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   NO
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
  pwm.setPWM(9, 0, Speed);     //Right rear wheel Reverse

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
* @brief         whistle
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
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
* Function       carRun
* @author        chengengyue
* @date          2019.10.08
* @brief         Input four motor speed to control car 
* @param[in1]    speed_L1 （-160~160）left front wheel
* @param[in2]    speed_R1 （-160~160）right front wheel
* @param[in3]    speed_L2 （-160~160）left rear wheel
* @param[in4]    speed_R2 （-160~160）right rear wheel
* @param[out]    void
* @par History   no
*/
void carRun(int speed_L1, int speed_R1, int speed_L2, int speed_R2)
{
  speed_L1 = map(speed_L1, -160, 160, -2560, 2560); //map 160 to 2560
  speed_R1 = map(speed_R1, -160, 160, -2560, 2560);
  speed_L2 = map(speed_L2, -160, 160, -2560, 2560);
  speed_R2 = map(speed_R2, -160, 160, -2560, 2560);

  int speed_wheel[4] = {speed_R1, speed_L1, speed_L2, speed_R2};

  for (int i = 0; i < 4; i++)
  {
    if (speed_wheel[i] >= 0)
    {
      pwm.setPWM(wheel[i][0], 0, speed_wheel[i]);
      pwm.setPWM(wheel[i][1], 0, 0);
    }
    else
    {
      pwm.setPWM(wheel[i][0], 0, 0);
      pwm.setPWM(wheel[i][1], 0, -speed_wheel[i]);
    }
  }
}

/**
* Function       carSurround
* @author        chengengyue
* @date          2019.10.08
* @brief         Surround Mode
* @param[in1]    speed 
* @param[in2]    direction，0-left，1-right
* @param[out]    void
* @retval        void
* @par History   no
*/
void carSurround(int speed, int direction)
{
	int speed_L1 = speed * 1.5;
	int speed_L2 = speed;
	int speed_R1 = speed * 1.5;
	int speed_R2 = speed;

	ir_L1 = analogRead(IR_SENSOR_L1) / 10;
	ir_L2 = analogRead(IR_SENSOR_L2) / 10;
	ir_R1 = analogRead(IR_SENSOR_R1) / 10;
	ir_R2 = analogRead(IR_SENSOR_R2) / 10;
	ir_Mid = analogRead(IR_SENSOR_MID) / 10;
	int pid_mid = PIDCal_IR(PID_MID, ir_Mid);

	if (direction == 0)            // left
	{
		if (dig_ir_l1 == LOW && dig_ir_r1 == HIGH)
		{
			speed_L1 = speed_L1 - ir_L1;
			speed_L2 = speed_L2 + ir_L1 - 40;
			speed_R1 = speed_R1 + ir_R1 - 40;
			speed_R2 = speed_R2 - ir_R1 + 40;
		}else if (dig_ir_l1 == HIGH && dig_ir_r1 == LOW)
		{
			speed_L1 = speed_L1 - ir_L1 + 40;
			speed_L2 = speed_L2 - ir_L1 + 40;
			speed_R1 = speed_R1 + ir_R1 - 40;
			speed_R2 = speed_R2 + ir_R1;
		}else if (dig_ir_l1 == LOW && dig_ir_r1 == LOW)
		{
			speed_L1 = speed_L1 - ir_L1 + 43;
			speed_L2 = speed_L2 + ir_L1 - 43;
			speed_R1 = speed_R1 + ir_R1 - 43;
			speed_R2 = speed_R2 - ir_R1 + 43;
		}
		
		carRun(-speed_L1 + pid_mid, speed_R1 + pid_mid, speed_L2 + pid_mid, -speed_R2 + pid_mid);
	}
	else if (direction == 1)      // right
	{
		if (dig_ir_l1 == LOW && dig_ir_r1 == HIGH)
		{
			speed_L1 = speed_L1 + ir_L1;
			speed_L2 = speed_L2 - ir_L1 + 40;
			speed_R1 = speed_R1 - ir_R1 + 40;
			speed_R2 = speed_R2 + ir_R1 - 40;
		}else if (dig_ir_l1 == HIGH && dig_ir_r1 == LOW)
		{
			speed_L1 = speed_L1 + ir_L1 - 40;
			speed_L2 = speed_L2 + ir_L1 - 40;
			speed_R1 = speed_R1 - ir_R1 + 40;
			speed_R2 = speed_R2 - ir_R1;
		}else if (dig_ir_l1 == LOW && dig_ir_r1 == LOW)
		{
			speed_L1 = speed_L1 + ir_L1 - 43;
			speed_L2 = speed_L2 - ir_L1 + 43;
			speed_R1 = speed_R1 - ir_R1 + 43;
			speed_R2 = speed_R2 + ir_R1 - 43;
		}
		
		carRun(speed_L1 + pid_mid, -speed_R1 + pid_mid, -speed_L2 + pid_mid, speed_R2 + pid_mid);
	}
}

/**
* Function       PIDCal_IR
* @author        chengengyue
* @date          2019.10.08
* @brief         PID calculation sensor value
* @param[in1]    pid
* @param[in2]    nowValue
* @param[out]    -val_sensor
* @retval        void
* @par History   no
*/
float PIDCal_IR(PID pid, float nowValue)
{
  float dError, Error;
  Error = pid.SetPoint - nowValue;        
  pid.SumError += Error;                 
  dError = pid.LastError - pid.PrevError; 
  pid.PrevError = pid.LastError;
  pid.LastError = Error;

  double val_sensor = pid.Proportion * Error    
             + pid.Integral * pid.SumError     
             + pid.Derivative * dError;      
  return -val_sensor;
}

/**
* Function       setup
* @author        chengengyue
* @date          2019.08.20
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
* @date          2019.08.20
* @brief         Parse the data sent from the serial port and execute the corresponding command
* @param[in]     void
* @retval        void
* @par History   no
*/
void loop()
{
  keyscan();
  if (button_press)
  {
    ir_rgb();      
    carSurround(CarSpeedControl, 0); 
  }
  else
  {
    brake();    
    clearRGB(); 
  }
}
