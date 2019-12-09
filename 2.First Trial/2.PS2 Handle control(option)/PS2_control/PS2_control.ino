/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         PS2_control.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        PS2 handle control omniduino car
* @details
* @par History  
*
*/
//Import library file
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <PS2X_lib.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define ENABLE_DEBUG_

#define PS2_DO_CMD 11
#define PS2_DI_DAT 12
#define PS2_CLK 13
#define PS2_CS_SEL 7

#define pressures true
#define rumble true

#define KEY_PIN 8		    //Define key pin 
#define INTERRUPT_PIN 2 //Define MPU6050 interrupt pin
#define LED_PIN 5		    //Define gyroscope status light pins
#define RGB_PIN 9		    //Define RGB pin 
#define MAX_LED 4		    //4 RGB lights
#define BUZZER 10       //Define buzzer pin 

//Initialize MPU6050, pca9685, RGB programming light, PS2
MPU6050 mpu = MPU6050(0x68);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_LED, RGB_PIN, NEO_RGB + NEO_KHZ800);
PS2X ps2x;

int color_RGB = 0;     // RGB color, 1-green，2-blue
int brightness = 150;  // RGB brightness 
int select = 0;        // Select button for selecting joystick or direction key control
int error = 0;         // Save the value initialized by the PS2 handle
byte type = 0;         // PS2 handle type
byte vibrate = 0;      // Vibration intensity value

// MPU CarSpeedControl/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorInt16 aa;		 // [x, y, z]            accel sensor measurements
VectorInt16 gy;		 // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];		 // [psi, theta, phi]    Euler angle container
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

/*小车初始速度控制*/
const char wheel[4][2] = {{10, 11}, {13, 12}, {15, 14}, {8, 9}};
float wheel_speed[4];
int CarSpeedControl = 100;



/*====================================================================================================
PID Function
The PID (Proportional、Integral、Desired) function is used in mainly
CarSpeedControl applications. PIDCalc performs one iteration of the PID
algorithm.
While the PID function works, main is just a dummy program showing
a typical usage.
=====================================================================================================*/
typedef struct
{
	float SetPoint;   // Desired value
	float Proportion; // Proportional Const
	float Integral;   // Integral Const
	float Derivative; // Derivative Const
	float LastError;  // Error[-1]
	float PrevError;  // Error[-2]
	float SumError;   // Sums of Errors
} PID;

/*====================================================================================================/
PID calculation section
=====================================================================================================*/
PID omega_PID = {0, 0.4, 0, 0.1, 0, 0, 0};
PID alpha_PID = {0, 0.1, 0.001, 0.001, 0, 0, 0};
float omega_Work = 0;

const typedef enum {
	enSTOP = 0,
	enRUN,
	enBACK,
	enLEFT,
	enRIGHT,
	enTLEFT,
	enTRIGHT,
	enUPLEFT,
	enUPRIGHT,
	enDOWNLEFT,
	enDOWNRIGHT
} enCarState;


int g_CarState = enSTOP; //1advance 2back 3left 4right 0stop



/**
* Function       mpu6050_getdata
* @author        chengengyue
* @date          2019.10.08
* @brief         Print PS2 controller receiver debug data
* @param[in1]	   error_code
* @param[in2]	   type_code
* @param[out]    void
* @par History   no
*/
void printPS2DebugData(int error_code, byte type_code)
{
	if (error_code == 1)
	{
		Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
	}
	else
	{
		if (error_code == 0)
		{
			Serial.print("Found Controller, configured successful ");
			Serial.print("pressures = ");
			if (pressures)
				Serial.println("true ");
			else
				Serial.println("false");
			Serial.print("rumble = ");
			if (rumble)
				Serial.println("true)");
			else
				Serial.println("false");
			Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
			Serial.println("Press SELECT button to switch buttons and rocker controls.");	
		}
		else if (error_code == 2)
			Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

		else if (error_code == 3)
			Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
		
		switch (type_code)
		{
		case 0:
			Serial.println("Unknown Controller type found ");
			break;
		case 1:
			Serial.println("DualShock Controller found ");
			break;
		case 2:
			Serial.println("GuitarHero Controller found ");
			break;
		case 3:
			Serial.println("Wireless Sony DualShock Controller found ");
			break;
		}
	}
}

/**
* Function       mpu6050_getdata
* @author        chengengyue
* @date          2019.10.08
* @brief         getdata
* @param[in1]	   void
* @param[out]    void
* @par History   no
*/
void mpu6050_getdata()
{
	// if programming failed, turn off LED8 and don't try to do anything
	if (!dmpReady)
	{
		digitalWrite(LED_PIN, HIGH); // turn off LED9
		return;
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		fifoCount = mpu.getFIFOCount();
		// Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		// fifoCount -= packetSize;

		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		mpu.dmpGetAccel(&aa, fifoBuffer);

		digitalWrite(LED_PIN, LOW); //turn on LED9
	}
}

/**
* Function       initMPU6050
* @author        chengengyue
* @date          2019.10.08
* @brief         MPU6050 Initialization 
* @param[in1]	   void
* @param[out]    void
* @par History   no
*/
void initMPU6050()
{
	mpu.initialize();

	// verify connection
	bool mpu_state = mpu.testConnection();

	#ifdef ENABLE_DEBUG
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu_state ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	#endif
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(51);
	mpu.setYGyroOffset(8);
	mpu.setZGyroOffset(21);
	mpu.setXAccelOffset(1150);
	mpu.setYAccelOffset(-50);
	mpu.setZAccelOffset(1060);

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);

		#ifdef ENABLE_DEBUG
		Serial.println();
		mpu.PrintActiveOffsets();
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		#endif

		// enable Arduino interrupt detection
		mpu.setDMPEnabled(true);

		#ifdef ENABLE_DEBUG
		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		Serial.println(F(")..."));
		#endif

		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		#ifdef ENABLE_DEBUG
		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		#endif

		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		digitalWrite(LED_PIN, LOW);    //Initialization conpleted , LED9 is on.
	}
	else
	{

		#ifdef ENABLE_DEBUG
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
		#endif

		digitalWrite(LED_PIN, HIGH);   //Initialization failed , LED9 is off.
	}
}

/**
* Function       brake
* @author        chengengyue
* @date          2019.10.08
* @brief         stop
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
* Function       clearRGB
* @author        chengengyue
* @date          2019.10.08
* @brief         close on board RGB
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
* @brief         control RGB
* @param[in1]    num  Select the serial number of the RGB. 
*                If it is greater than or equal to the maximum number of the RGB, it will be fully illuminated.
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

/*
* Function       boot_animation
* @author        chengengyue
* @date          2019.10.08
* @brief         The green light is gradually lit 
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void boot_animation()
{
  uint32_t color = strip.Color(0, 0, 0);
  for (int a = 0; a < 256; a += 2)
  {
    color = strip.Color(a, 0, 0);
    for (uint8_t i = 0; i < 4; i++)
    {
      strip.setPixelColor(i, color);
    }
    strip.show();
    delay(20);
  }
}

/*
* Function       adjust_light
* @author        chengengyue
* @date          2019.10.08
* @brief         Adjust brightness
* @param[in1]    num    Select the serial number of the RGB. 
*                       If it is greater than or equal to the maximum number of the RGB, it will be fully illuminated.
* @param[in2]    color  select color 0-red，1-green，2-blue
* @param[in3]    bright 0~255
* @param[out]    void
* @retval        void
* @par History   np
*/
void adjust_light(int num, int color, int bright)
{
	uint32_t color_strip = strip.Color(0, 0, 0);
	if (color == 0)
	{
		color_strip = strip.Color(0, bright, 0);
	}else if (color == 1)
	{
		color_strip = strip.Color(bright, 0, 0);
	}else if (color == 2)
	{
		color_strip = strip.Color(0, 0, bright);
	}
	
	if (num < MAX_LED)
	{
		strip.setPixelColor(num, color_strip);
	}else
	{
		for (int i = 0; i < MAX_LED; i++)
		{
			strip.setPixelColor(i, color_strip);
		}
	}
	strip.show();
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
		digitalWrite(BUZZER, LOW);  // no sound
		delay(1);					
	}
}

/**
* Function       carRun
* @author        chengengyue
* @date          2019.10.08
* @brief         Input 4 motor value to control movement of car
* @param[in1]	   speed_L1 （-160~160）
* @param[in2]	   speed_R1 （-160~160）
* @param[in3]	   speed_L2 （-160~160）
* @param[in4]	   speed_R2 （-160~160）
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
* Function       mecanum_run
* @author        chengengyue
* @date          2019.10.08
* @brief         mecanum  movement
* @param[in1]    car_alpha 
* @param[in2]    speed_L   
* @param[in3]    car_omega 
* @param[in4]    speed_A   
* @param[out]    void
* @retval        void
* @par History   no
*/
void mecanum_run(float car_alpha, int speed_L, float car_omega, int speed_A)
{
	speed_L = speed_L * 16;
	speed_A = speed_A * 16;

	float speed_x = speed_L * sin(car_alpha);
	float speed_y = speed_L * cos(car_alpha);

	float speed_omega = float(speed_A * sin(car_omega));

	wheel_speed[0] = speed_y - speed_x + speed_omega;
	wheel_speed[1] = speed_y + speed_x - speed_omega;
	wheel_speed[2] = speed_y - speed_x - speed_omega;
	wheel_speed[3] = speed_y + speed_x + speed_omega;

	for (int i = 0; i < 4; i++)
	{
		if (wheel_speed[i] >= 0)
		{
			pwm.setPWM(wheel[i][0], 0, wheel_speed[i]);
			pwm.setPWM(wheel[i][1], 0, 0);
		}
		else
		{
			pwm.setPWM(wheel[i][0], 0, 0);
			pwm.setPWM(wheel[i][1], 0, -wheel_speed[i]);
		}
	}
}

/**
* Function       mecanum_run_fb
* @author        chengengyue
* @date          2019.06.25
* @brief         car move forward and back
* @param[in1]    car_alpha 
* @param[in2]    speed_L   
* @param[in3]    car_omega 
* @param[in4]    speed_A   
* @param[out]    void
* @retval        void
* @par History   no
*/
void mecanum_run_fb(float car_alpha, int speed_L, float car_omega, int speed_A)
{
	speed_L = speed_L * 16;
	speed_A = speed_A * 16;

	float speed_x = speed_L * sin(car_alpha);
	float speed_y = speed_L * cos(car_alpha);

	float speed_omega = float(speed_A * sin(car_omega));

	wheel_speed[0] = speed_y - speed_x - speed_omega;
	wheel_speed[1] = speed_y + speed_x + speed_omega;
	wheel_speed[2] = speed_y - speed_x;
	wheel_speed[3] = speed_y + speed_x;

	for (int i = 0; i < 4; i++)
	{
		if (wheel_speed[i] >= 0)
		{
			pwm.setPWM(wheel[i][0], 0, wheel_speed[i]);
			pwm.setPWM(wheel[i][1], 0, 0);
		}
		else
		{
			pwm.setPWM(wheel[i][0], 0, 0);
			pwm.setPWM(wheel[i][1], 0, -wheel_speed[i]);
		}
	}
}

float cal_omega(float a)
{
	return (a * 180 / M_PI); 
}

/**
* Function       PIDCalc
* @author        chengengyue
* @date          2019.10.08
* @brief         PID Calculate the yaw angle
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   no
*/
float PIDCalc(float NextPoint)
{
	float dError, Error;
	Error = omega_PID.SetPoint - NextPoint;				//Deviation
	omega_PID.SumError += Error;						//Integral
	dError = omega_PID.LastError - omega_PID.PrevError; //Current differential
	omega_PID.PrevError = omega_PID.LastError;
	omega_PID.LastError = Error;

	double omega_rad = omega_PID.Proportion * Error				 //Proportional term
					   + omega_PID.Integral * omega_PID.SumError  //Integral term
					   + omega_PID.Derivative * dError;			      //Differential term

	if (omega_rad > M_PI / 6)
		omega_rad = M_PI / 6;
	if (omega_rad < -M_PI / 6)
		omega_rad = -M_PI / 6;
	return -omega_rad;
}

/**
* Function       PS2_Ctrol
* @author        chengengyue
* @date          2019.10.08
* @brief         PID Calculate the value of the sensor
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   no
*/
void PS2_Ctrol(void)
{
	int X1, Y1, X2, Y2;
	if (error == 1) 
	{
		error = ps2x.config_gamepad(PS2_CLK, PS2_DO_CMD, PS2_CS_SEL, PS2_DI_DAT, pressures, rumble);
		type = ps2x.readType();

		#ifdef ENABLE_DEBUG
		Serial.println("waiting for controller connect");
		printPS2DebugData(error, type);
		#endif

		delay(500);
		return;
	}

	if (type != 1) //skip loop if no controller found
		return;

	//DualShock Controller
	ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

	if (ps2x.ButtonPressed(PSB_START)) //will be TRUE as long as button is pressed
	{
		#ifdef ENABLE_DEBUG
		Serial.println("Start is pressed");
		#endif
	}
	if (ps2x.ButtonPressed(PSB_SELECT))
	{
		select = !select;
		g_CarState = enSTOP;
		if (select == 0)
		{
			showRGB(4, 0, brightness, 0);
			color_RGB = 1;
		}
		else
		{
			showRGB(4, 0, 0, brightness);
			color_RGB = 2;
		}

		#ifdef ENABLE_DEBUG
		Serial.print("Select is pressed...Select = ");
		Serial.println(select);
		#endif
	}

	if (select == 0)
	{
		if (ps2x.Button(PSB_PAD_UP))
		{ //will be TRUE as long as button is pressed
			#ifdef ENABLE_DEBUG
			Serial.print("Up held this hard: ");
			Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
			#endif

			g_CarState = enRUN;
		}
		else if (ps2x.Button(PSB_PAD_RIGHT))
		{
			#ifdef ENABLE_DEBUG
			Serial.print("Right held this hard: ");
			Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
			#endif

			g_CarState = enRIGHT;
		}
		else if (ps2x.Button(PSB_PAD_LEFT))
		{
			#ifdef ENABLE_DEBUG
			Serial.print("LEFT held this hard: ");
			Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
			#endif

			g_CarState = enLEFT;
		}
		else if (ps2x.Button(PSB_PAD_DOWN))
		{
			#ifdef ENABLE_DEBUG
			Serial.print("DOWN held this hard: ");
			Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
			#endif

			g_CarState = enBACK;
		}
		else if (ps2x.Button(PSB_SQUARE)) //spin left with square 
		{ 
			#ifdef ENABLE_DEBUG
			Serial.println("Square just pressed");
			#endif

			g_CarState = enTLEFT;
		}
		else if (ps2x.Button(PSB_CIRCLE)) //spin right with circle            //will be TRUE if button was JUST pressed
		{
			#ifdef ENABLE_DEBUG
			Serial.println("Circle just pressed");
			#endif
			g_CarState = enTRIGHT;
		}
		else
		{
			g_CarState = enSTOP;
		}
	}
	else
	{
		#ifdef ENABLE_DEBUG
		Serial.print("Stick Values:");
		Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
		Serial.print(",");
		Serial.print(ps2x.Analog(PSS_LX), DEC);
		Serial.print(",");
		Serial.print(ps2x.Analog(PSS_RY), DEC);
		Serial.print(",");
		Serial.println(ps2x.Analog(PSS_RX), DEC);
		#endif

		Y1 = map(ps2x.Analog(PSS_LY), 255, 0, 0, 255);
		X1 = ps2x.Analog(PSS_LX);
		// Y2 = map(ps2x.Analog(PSS_RY), 255, 0, 0, 255);
		X2 = ps2x.Analog(PSS_RX);

		ps2_carRun(X1, Y1, X2);
	}

	//this will set the large motor vibrate speed based on how hard you press the blue (X) button
	vibrate = ps2x.Analog(PSAB_CROSS);

	if (ps2x.NewButtonState())
	{
		//will be TRUE if any button changes state (on to off, or off to on)
		if (ps2x.Button(PSB_R1)) // Accelerate, add 20 each time
		{
			CarSpeedControl += 20;
			if (CarSpeedControl >= 160)
				CarSpeedControl = 160;
			#ifdef ENABLE_DEBUG
			Serial.print("R1 pressed! Speed=");
			Serial.println(CarSpeedControl);
			#endif
		}
		if (ps2x.Button(PSB_R2)) //Slow down, minus 20 each time
		{
			CarSpeedControl -= 20;
			if (CarSpeedControl <= 60)
				CarSpeedControl = 60;
			#ifdef ENABLE_DEBUG
			Serial.print("R2 pressed! Speed=");
			Serial.println(CarSpeedControl);
			#endif
		}
		if (ps2x.Button(PSB_TRIANGLE))    //Triangle
		{
			whistle();

			#ifdef ENABLE_DEBUG
			Serial.println("Triangle pressed");
			#endif
		}
	}

	if (ps2x.Button(PSB_CROSS))          // X
	{
		brightness += 3;
		if (brightness >= 255)
		{
			brightness = 255;
			if (ps2x.ButtonPressed(PSB_CROSS))
				brightness = 0;
		}
		
		adjust_light(MAX_LED, color_RGB, brightness);
		#ifdef ENABLE_DEBUG
		Serial.print(brightness);
		Serial.println("\tX just changed");
		#endif
	}
}

/**
* Function       ps2_carRun
* @author        chengengyue
* @date          2019.10.08
* @brief         ps2 handle rocker control car
* @param[in1]     x (0~255) control X direction speed 
* @param[in2]    y (0~255)  control Y direction speed
* @param[in3]    spin (0~255) 
* @param[out]    void
* @par History   no
*/
void ps2_carRun(int x, int y, int spin)
{
  int speed_yaw_b = 0, speed_yaw_f = 0;

  if (abs(x - 128) < 30 && abs(y - 128) < 30)
  {
    x = 128;
    y = 128;
    omega_PID.SetPoint = ypr[0];
  }else if (abs(x - 128) > 30 && abs(y - 128) < 30)
  {
    speed_yaw_f = CarSpeedControl * sin(omega_Work);
    speed_yaw_b = speed_yaw_f;
  }
  else if (abs(x - 128) < 30 && abs(y - 128) > 30)
  {
    speed_yaw_f = CarSpeedControl * sin(omega_Work);
    speed_yaw_b = 0;
  }
  if (abs(spin - 128) < 30)
  {
    spin = 128;
  }
  
  int speed_X = map(x, 0, 255, -CarSpeedControl, CarSpeedControl);
  int speed_Y = map(y, 0, 255, -CarSpeedControl, CarSpeedControl);
  int spin_A = map(spin, 0, 255, -CarSpeedControl * 0.8, CarSpeedControl * 0.8);

  int speed_L1 = speed_Y + speed_X + spin_A - speed_yaw_f;
  int speed_L2 = speed_Y - speed_X + spin_A - speed_yaw_b;
  int speed_R1 = speed_Y - speed_X - spin_A + speed_yaw_f;
  int speed_R2 = speed_Y + speed_X - spin_A + speed_yaw_b;

  carRun(speed_L1, speed_R1, speed_L2, speed_R2);
  delay(10);
}

/**
* Function       setup
* @author        chengengyue
* @date          2019.10.08
* @brief         Initial configuration
* @param[in]     void
* @retval        void
* @par History   no
*/
void setup()
{
  pinMode(INTERRUPT_PIN, INPUT); 
  pinMode(LED_PIN, OUTPUT);      
  pinMode(BUZZER, OUTPUT);       

  Serial.begin(9600);

  strip.begin();
  strip.show();
  clearRGB();

  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  brake();            

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  digitalWrite(LED_PIN, HIGH); //Close the gyroscope status indicator LED9
  initMPU6050();               //Initial MPU6050

  /*Initial PS2*/
  error = ps2x.config_gamepad(PS2_CLK, PS2_DO_CMD, PS2_CS_SEL, PS2_DI_DAT, pressures, rumble);
  type = ps2x.readType();

  #ifdef ENABLE_DEBUG
  printPS2DebugData(error, type);
  #endif

  delay(1000);
  boot_animation();
  color_RGB = 1;
}

/**
* Function       loop
* @author        chengengyue
* @date          2019.10.08
* @brief         mian function
* @param[in]     void
* @retval        void
* @par History   no
*/
void loop()
{
  mpu6050_getdata();
  omega_Work = PIDCalc(ypr[0]);

  PS2_Ctrol();

  // g_CarState = 3;
  if (select == 0)
  {
    switch (g_CarState)
    {
    case enSTOP:
      omega_PID.SetPoint = ypr[0];
      brake();
      break;
    case enRUN:
      mecanum_run_fb(0, CarSpeedControl * 3 / 4, -omega_Work, CarSpeedControl * 3 / 4);
      delay(20);
      break;
    case enLEFT:
      mecanum_run(-M_PI / 2, CarSpeedControl, omega_Work, CarSpeedControl);
      delay(10);
      break;
    case enRIGHT:
      mecanum_run(M_PI / 2, CarSpeedControl, omega_Work, CarSpeedControl);
      delay(10);
      break;
    case enBACK:
      mecanum_run_fb(M_PI, CarSpeedControl * 3 / 4, -omega_Work, CarSpeedControl * 3 / 4);
      delay(20);
      break;
    case enTLEFT:
      mecanum_run(0, 0, M_PI / 2, CarSpeedControl * 3 / 4);
      break;
    case enTRIGHT:
      mecanum_run(0, 0, -M_PI / 2, CarSpeedControl * 3 / 4);
      break;
    default:
      omega_PID.SetPoint = ypr[0];
      brake();
      break;
    }
    delay(10);
  }
}
