/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         omniduino APP control.ino
* @author       chengengyue
* @version      V1.1
* @date         2019.12.03
* @brief        omniduino APP control
* @details
* @par History  
* V1.1 adds car self-stabilization function
*/
//Import library file
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define ENABLE_DEBUG1

#define BUZZER 10		//Define buzzer pins
#define KEY_PIN 8		//Define button pins
#define INTERRUPT_PIN 2 //Define MPU6050 pins
#define LED_PIN 5		//Define status indiactor  pins
#define RGB_PIN 9		//Define RGB pins
#define MAX_LED 4		//Car with 4 RGB lights

#define IR_SENSOR_L1 A3
#define IR_SENSOR_L2 A0
#define IR_SENSOR_R1 A2
#define IR_SENSOR_R2 A1
#define IR_SENSOR_MID A7

/* Tone */
#define G5 392   
#define A6 440   
#define B7 494   
#define c1 525   
#define d2 587   
#define e3 659   
#define f4 698   
#define g5 784   
#define a6 880   
#define b7 988   
#define C1 1047 
#define D2 1175  
#define E3 1319  
#define F4 1397  
#define GG5 1568 
#define AA6 1769 
#define g4 392
#define c5 523
#define a4 440
#define d5 587
#define e5 659
#define b4 494
#define c6 1047
#define d6 1175
#define b5 988
#define a5 880
#define g5 784
#define e6 1319
#define f6 1397
#define a5 880
#define f5 698

//MPU6050、pca9685、RGB
MPU6050 mpu = MPU6050(0x68);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_LED, RGB_PIN, NEO_RGB + NEO_KHZ800);

// MPU control/status vars
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

const char wheel[4][2] = {{10, 11}, {13, 12}, {15, 14}, {8, 9}};
int CarSpeedControl = 40;
int Rocker_X = 128;
int Rocker_Y = 128;
int spin = 0;

int carDir = 0; //Car self-stabilizing adjustment flag, 0 is adjustment (stop), 1 is non-adjustment (remote control)

unsigned long oldTime = 0;
unsigned long nowTime = 0;

//button state
bool button_press = false;

/*====================================================================================================
PID Function
The PID (Proportional, integral, differential) function is used in mainly
control applications. PIDCalc performs one iteration of the PID
algorithm.
While the PID function works, main is just a dummy program showing
a typical usage.
=====================================================================================================*/
typedef struct
{
	float SetPoint;   // Set target Desired value
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
PID alpha_PID = {0, 5, 0, 0.1, 0, 0, 0};
PID omega_PID = {0, 0.4, 0, 0.1, 0, 0, 0};
PID PID_MID = {60, 0.5, 0, 0, 0, 0, 0};
PID PID_IR = {40, 1.2, 0, 0, 0, 0, 0};
float PIDCal_IR(PID pid, float nowValue);
float PIDCal_Stabilize(PID pid, float nowValue);
float alpha_Work = 0;
float omega_Work = 0;

/*Serial data setting*/
int IncomingByte = 0;			 //Received  data byte
int Receive_Length = 0;			 //Length of data
String InputString = "";		 //Used to store received content
boolean NewLineReceived = false; //Previous data end mark
boolean StartBit = false;		 //Agreement start sign

/*Car mode select*/
int g_modeSelect = 0; //0-default state; 1-Surround mode 2-Avoid mode 3-Pan mode

/* RGBlight effect */
int RGB_Effect = 0; //1-breathing light, 2-marquee, 3-water light, 4-colorful light, 5-close light

/*Buzzer related parameters*/
int buzzer_state = 0;									 //Buzzer status
int buzzer_music = 0;									 //select song 
int music_index = 0;									 //Music progress
const unsigned char music_max[5] = {42, 39, 36, 70, 21}; //Maximum length of all songs

enum enMusic
{
	enLittleStar = 1,
	enBingo,
	enMerryChristmas,
	enOdeToJoy,
	enBirthday
};

/*Define variables to save the size of the data collected by the infrared sensor*/
int ir_L1;  //left front 
int ir_L2;  //left rear
int ir_R1;  //right front
int ir_R2;  //right rear
int ir_Mid; //Middle front

/*Define variables to save whether the infrared sensor detects an obstacle*/
int dig_ir_l1; 
int dig_ir_l2; 
int dig_ir_r1; 
int dig_ir_r2; 

int car_direction = 0;   //Set the direction of the surround mode, 0 is left and 1 is right
int state_direction = 0; //Set the direction of the pan mode, 0 is left and 1 is right
int state_ontrack = 0;   //Indicates whether the car is in the track, 0 is out, 1 is in

const PROGMEM int tone_Birthday[21][2]{
	{G5, 2}, {A6, 2}, {G5, 2}, {c1, 2}, {B7, 4}, {G5, 2}, {A6, 2}, {G5, 2}, {d2, 2}, {c1, 4}, {G5, 2}, {g5, 2}, {e3, 2}, {c1, 2}, {B7, 2}, {A6, 2}, {f4, 2}, {e3, 2}, {c1, 2}, {d2, 2}, {c1, 2}};

const PROGMEM int tone_OdeToJoy[70][2]{
	{e3, 2}, {e3, 2}, {f4, 2}, {g5, 2}, {g5, 2}, {f4, 2}, {e3, 2}, {d2, 2}, {c1, 2}, {c1, 2}, {d2, 2}, {e3, 2}, {e3, 3}, {d2, 1}, {d2, 4}, {e3, 2}, {e3, 2}, {f4, 2}, {g5, 2}, {g5, 2}, {f4, 2}, {e3, 2}, {d2, 2}, {c1, 2}, {c1, 2}, {d2, 2}, {e3, 2}, {d2, 3}, {c1, 1}, {c1, 4}, {d2, 2}, {d2, 2}, {e3, 2}, {c1, 2}, {d2, 2}, {e3, 1}, {f4, 1}, {e3, 2}, {c1, 2}, {d2, 2}, {e3, 1}, {f4, 1}, {e3, 2}, {d2, 2}, {c1, 2}, {d2, 2}, {G5, 2}, {e3, 2}, {e3, 2}, {f4, 2}, {g5, 2}, {g5, 2}, {f4, 2}, {e3, 2}, {d2, 2}, {c1, 2}, {c1, 2}, {d2, 2}, {e3, 2}, {d2, 3}, {c1, 1}, {c1, 4}};

const PROGMEM int tone_LittleStar[42][2]{
	{c1, 2}, {c1, 2}, {g5, 2}, {g5, 2}, {a6, 2}, {a6, 2}, {g5, 4}, {f4, 2}, {f4, 2}, {e3, 2}, {e3, 2}, {d2, 2}, {d2, 2}, {c1, 4}, {g5, 2}, {g5, 2}, {f4, 2}, {f4, 2}, {e3, 2}, {e3, 2}, {d2, 4}, {g5, 2}, {g5, 2}, {f4, 2}, {f4, 2}, {e3, 2}, {e3, 2}, {d2, 4}, {c1, 2}, {c1, 2}, {g5, 2}, {g5, 2}, {a6, 2}, {a6, 2}, {g5, 4}, {f4, 2}, {f4, 2}, {e3, 2}, {e3, 2}, {d2, 2}, {d2, 2}, {c1, 4}};

const PROGMEM int tone_MerryChristmas[36][2]{
	{g5, 1}, {g5, 1}, {c6, 2}, {c6, 1}, {d6, 1}, {c6, 1}, {b5, 1}, {a5, 2}, {a5, 2}, {0, 2}, {a5, 1}, {a5, 1}, {d6, 2}, {d6, 1}, {e6, 1}, {d6, 1}, {c6, 1}, {b5, 2}, {g5, 2}, {0, 2}, {g5, 1}, {g5, 1}, {e6, 2}, {e6, 1}, {f6, 1}, {e6, 1}, {d6, 1}, {c6, 2}, {a5, 2}, {0, 2}, {g5, 1}, {g5, 1}, {a5, 1}, {d6, 1}, {b5, 1}, {c6, 2}};

const PROGMEM int tone_Bingo[39][2]{
	{g4, 1}, {c5, 1}, {c5, 1}, {c5, 1}, {g4, 1}, {a4, 1}, {a4, 1}, {g4, 1}, {g4, 1}, {c5, 1}, {c5, 1}, {d5, 1}, {d5, 1}, {e5, 2}, {c5, 1}, {0, 1}, {e5, 2}, {e5, 2}, {f5, 1}, {f5, 1}, {f5, 2}, {d5, 2}, {d5, 2}, {e5, 1}, {e5, 1}, {e5, 2}, {c5, 2}, {c5, 2}, {d5, 1}, {d5, 1}, {d5, 1}, {c5, 1}, {b4, 1}, {g4, 1}, {a4, 1}, {b4, 1}, {c5, 2}, {c5, 1}, {c5, 1}};

/*Servo related parameters*/
int servo_degree = 90;

/**
* Function       mpu6050_getdata
* @author        chengengyue
* @date          2019.10.08
* @brief         Read Mpu6050 Data
* @param[in1]	   void
* @param[out]    void
* @par History   no
*/
void mpu6050_getdata()
{
	// if programming failed, turn off LED9 and don't try to do anything
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

#ifdef ENABLE_DEBUG
		Serial.print(ypr[0] * 180 / M_PI);
		Serial.println("");
#endif

		digitalWrite(LED_PIN, LOW); //turn on LED9
	}
}

/**
* Function       initMPU6050
* @author        chengengyue
* @date          2019.10.08
* @brief         MPU6050
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
		digitalWrite(LED_PIN, LOW); //Initialization completed, turn on LED D9 indicator
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

		digitalWrite(LED_PIN, HIGH); //Initialization failed, turn off the LED D9 indicator
	}
}

/**
* Function       run
* @author        chengengyue
* @date          2019.10.08
* @brief         car advance
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void run(int Speed)
{
	Speed = map(Speed, 0, 160, 0, 2560);
	pwm.setPWM(10, 0, Speed); //Right rear wheel Forward
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, Speed); //Right front wheel Forward
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, Speed); //Left rear wheel Forward
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, Speed); //Left front wheel Forward
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
	pwm.setPWM(11, 0, Speed); //Right rear wheel Reverse
	pwm.setPWM(8, 0, 0);
	pwm.setPWM(9, 0, Speed); //Right front wheel Reverse

	pwm.setPWM(13, 0, 0);
	pwm.setPWM(12, 0, Speed); //Left rear wheel Reverse
	pwm.setPWM(15, 0, 0);
	pwm.setPWM(14, 0, Speed); //Left front wheel Reverse
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
	pwm.setPWM(10, 0, Speed); //Right rear wheel(B type) Forward
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, 0);
	pwm.setPWM(9, 0, Speed); //Right front wheel(A type) Reverse

	pwm.setPWM(13, 0, 0);
	pwm.setPWM(12, 0, Speed); //Left rear wheel (A type) Reverse
	pwm.setPWM(15, 0, Speed); //Left front wheel(B type) Forward
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
	pwm.setPWM(11, 0, Speed); //Right front wheel(B type) Reverse
	pwm.setPWM(8, 0, Speed);  //Right rear wheel(A type) Forward
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, Speed); //Left front wheel(A type) Forward
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, 0);
	pwm.setPWM(14, 0, Speed); //Left rear wheel(B type) Reverse
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
	pwm.setPWM(10, 0, Speed); //Right front wheel Forward
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, Speed); //Right rear wheel Forward
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, 0);
	pwm.setPWM(12, 0, Speed); //Left front wheel  Reserve
	pwm.setPWM(15, 0, 0);
	pwm.setPWM(14, 0, Speed); //Left rear wheel  Reserve
}

/**
* Function       spin_right
* @author        chengengyue
* @date          2019.10.08
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
	pwm.setPWM(11, 0, Speed); //Right front wheel Reserve
	pwm.setPWM(8, 0, 0);
	pwm.setPWM(9, 0, Speed); //Right rear wheel Reserve

	pwm.setPWM(13, 0, Speed); //Left front wheel Forward
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, Speed); //Left rear wheel Forward
	pwm.setPWM(14, 0, 0);
}

/**
* Function       front_left
* @author        chengengyue
* @date          2019.10.08
* @brief         car moves to the left front (A type wheel stop，B type wheel Forward)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void front_left(int Speed)
{
	Speed = map(Speed, 0, 160, 0, 2560);
	pwm.setPWM(10, 0, Speed); //Right front wheel(B type) Forward
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, 0);     //Right rear wheel(A type) stop
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, 0);    //Left front wheel(A type) stop
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, Speed); //Left rear wheel(B type) Forward
	pwm.setPWM(14, 0, 0);
}

/**
* Function       front_right
* @author        chengengyue
* @date          2019.10.08
* @brief         car moves to the right front (A type wheel Forward，B type wheel stop)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void front_right(int Speed)
{
	Speed = map(Speed, 0, 160, 0, 2560);
	pwm.setPWM(10, 0, 0); //Right front wheel(B type) stop
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, Speed); //Right rear wheel(A type) Forward
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, Speed); //Left front wheel(A type) Forward
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, 0); //Left rear wheel(B type) stop
	pwm.setPWM(14, 0, 0);
}

/**
* Function       left_rear
* @author        chengengyue
* @date          2019.10.08
* @brief         car moves to the left rear (A type wheel Reserve，B type wheel stop)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void left_rear(int Speed)
{
	Speed = map(Speed, 0, 160, 0, 2560);
	pwm.setPWM(10, 0, 0); //Right front wheel(B type) stop
	pwm.setPWM(11, 0, 0);
	pwm.setPWM(8, 0, 0);
	pwm.setPWM(9, 0, Speed); //Right rear wheel(A type) Reserve

	pwm.setPWM(13, 0, 0);
	pwm.setPWM(12, 0, Speed); //Left front wheel(A type) Reserve
	pwm.setPWM(15, 0, 0);	 //Left rear wheel(B type) stop
	pwm.setPWM(14, 0, 0);
}

/**
* Function       right_rear
* @author        chengengyue
* @date          2019.10.08
* @brief         car moves to the right rear (A type wheel stop，B type wheel Reserve)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void right_rear(int Speed)
{
	Speed = map(Speed, 0, 160, 0, 2560);
	pwm.setPWM(10, 0, 0);
	pwm.setPWM(11, 0, Speed); //Right front wheel(B type) Reserve
	pwm.setPWM(8, 0, 0);	    //Right rear wheel(A type) stop
	pwm.setPWM(9, 0, 0);

	pwm.setPWM(13, 0, 0);    //Left front wheel (A type) stop
	pwm.setPWM(12, 0, 0);
	pwm.setPWM(15, 0, 0);
	pwm.setPWM(14, 0, Speed); //Left rear wheel(B type) Reserve
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
* @brief         control RGB light
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
	if (num > MAX_LED - 1) //All RGB be illuminated
	{
		for (int i = 0; i < MAX_LED; i++)
		{
			strip.setPixelColor(i, color);
		}
	}
	else //Lighting a RGB separately
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

/**
* Function       waterfall_RGB
* @author        chengengyue
* @date          2019.10.08
* @brief          Water light（green）
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void waterfall_RGB()
{
	static int w = 0;
	clearRGB();
	showRGB(w, 0, 255, 0);
	w++;
	if (w >= 4)
		w = 0;
}

/**
* Function       marquee_RGB
* @author        chengengyue
* @date          2019.10.08
* @brief         Marquee
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void marquee_RGB()
{
	static int m = 0;
	clearRGB();
	switch (m)
	{
	case 0:
		showRGB(m % 4, 255, 0, 0);
		break;
	case 1:
		showRGB(m % 4, 255, 90, 0);
		break;
	case 2:
		showRGB(m % 4, 255, 255, 0);
		break;
	case 3:
		showRGB(m % 4, 0, 255, 0);
		break;
	case 4:
		showRGB(m % 4, 0, 255, 255);
		break;
	case 5:
		showRGB(m % 4, 0, 0, 255);
		break;
	case 6:
		showRGB(m % 4, 255, 0, 255);
		break;
	case 7:
		showRGB(m % 4, 255, 255, 255);
		break;

	default:
		break;
	}
	m++;
	if (m >= 8)
		m = 0;
}

/**
* Function       breathing_RGB
* @author        chengengyue
* @date          2019.10.08
* @brief         Breathing light
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void breathing_RGB()
{
	static int a = 0, b = 0, c = 0;
	if (a == 0)
	{
		b += 2;
		if (b >= 250)
			a = 1;
	}
	else
	{
		b -= 2;
		if (b <= 5)
		{
			a = 0;
			c++;
		}
	}
	switch (c)
	{
	case 0:
		showRGB(MAX_LED, b, 0, 0);
		break;
	case 1:
		showRGB(MAX_LED, b, b, 0);
		break;
	case 2:
		showRGB(MAX_LED, 0, b, 0);
		break;
	case 3:
		showRGB(MAX_LED, 0, b, b);
		break;
	case 4:
		showRGB(MAX_LED, 0, 0, b);
		break;
	case 5:
		showRGB(MAX_LED, b, 0, b);
		break;
	case 6:
		showRGB(MAX_LED, b, b, b);
		break;

	default:
		break;
	}
	c %= 7;
}

/**
* Function       colorfull_RGB
* @author        chengengyue
* @date          2019.10.08
* @brief         colorful RGB
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void colorfull_RGB()
{
	static int rgb_r = 254;
	static int rgb_g = 0;
	static int rgb_b = 0;
	if (rgb_r == 254 && rgb_g < 254 && rgb_b == 0)
		rgb_g += 2;
	else if (rgb_g == 254 && rgb_r > 0 && rgb_b == 0)
		rgb_r -= 2;
	else if (rgb_g == 254 && rgb_b < 254 && rgb_r == 0)
		rgb_b += 2;
	else if (rgb_b == 254 && rgb_g > 0 && rgb_r == 0)
		rgb_g -= 2;
	else if (rgb_b == 254 && rgb_r < 254 && rgb_g == 0)
		rgb_r += 2;
	else if (rgb_r == 254 && rgb_b > 0 && rgb_g == 0)
		rgb_b -= 2;

	showRGB(MAX_LED, rgb_r, rgb_g, rgb_b);
}

/**
* Function       bingo
* @author        chengengyue
* @date          2019.10.08
* @brief          play music：bingo
* @param[in]     index: Play progress
* @param[out]    void
* @retval        void
* @par History   no
*/
void bingo(int index)
{
	setBuzzer_Tone(pgm_read_word_near(&tone_Bingo[index][0]),
				   pgm_read_word_near(&tone_Bingo[index][1]));
}

/**
* Function       odeToJoy
* @author        chengengyue
* @date          2019.10.08
* @brief         play music：ode
* @param[in]     index: Play progress
* @param[out]    void
* @retval        void
* @par History   no
*/
void odeToJoy(int index)
{
	setBuzzer_Tone(pgm_read_word_near(&tone_OdeToJoy[index][0]),
				   pgm_read_word_near(&tone_OdeToJoy[index][1]));
}

/**
* Function       merryChristmas
* @author        chengengyue
* @date          2019.10.08
* @brief         play music：Merry Christmas
* @param[in]     index: Play progress
* @param[out]    void
* @retval        void
* @par History   no
*/
void merryChristmas(int index)
{
	setBuzzer_Tone(pgm_read_word_near(&tone_MerryChristmas[index][0]),
				   pgm_read_word_near(&tone_MerryChristmas[index][1]));
}

/**
* Function       littleStar
* @author        chengengyue
* @date          2019.10.08
* @brief         play music：star
* @param[in]     index:Play progress
* @param[out]    void
* @retval        void
* @par History   no
*/
void littleStar(int index)
{
	setBuzzer_Tone(pgm_read_word_near(&tone_LittleStar[index][0]),
				   pgm_read_word_near(&tone_LittleStar[index][1]));
}

/**
* Function       birthday
* @author        chengengyue
* @date          2019.10.08
* @brief         play music：birthday
* @param[in]     index:Play progress
* @param[out]    void
* @retval        void
* @par History   no
*/
void birthday(int index)
{
	setBuzzer_Tone(pgm_read_word_near(&tone_Birthday[index][0]),
				   pgm_read_word_near(&tone_Birthday[index][1]));
}

/**
* Function       setBuzzer_Tone
* @author        chengengyue
* @date          2019.10.08
* @brief         The buzzer sounds at a certain frequency
* @param[in1]    frequency  
* @param[in2]    duration   
* @param[out]    void
* @retval        void
* @par History   no
*/
void setBuzzer_Tone(uint16_t frequency, uint32_t duration)
{
	int period = 1000000L / frequency; //1000000L
	int pulse = period / 2;
	for (long i = 0; i < duration * 200000L; i += period)
	{
		digitalWrite(BUZZER, HIGH);
		delayMicroseconds(pulse);
		digitalWrite(BUZZER, LOW);
		delayMicroseconds(pulse);
	}
	if (frequency == 0)
		delay(230 * duration);
	delay(20);
}

/**
* Function       music_play
* @author        chengengyue
* @date          2019.10.08
* @brief         buzzer play music
* @param[in1]    music  
* @param[in2]    index  
* @param[out]    void
* @retval        void
* @par History   no
*/
void music_play(int music, int index)
{
	switch (music)
	{
	case enLittleStar:
		littleStar(index);
		break;

	case enBingo:
		bingo(index);
		break;

	case enMerryChristmas:
		merryChristmas(index);
		break;

	case enOdeToJoy:
		odeToJoy(index);
		break;

	case enBirthday:
		birthday(index);
		break;

	default:
		break;
	}
}

/**
* Function       ir_rgb
* @author        chengengyue
* @date          2019.10.08
* @brief         Which sensor detected the obstacle and the RGB light next to it lights up red.
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
* Function       carSurround
* @author        chengengyue
* @date          2019.10.07
* @brief         car Surround 
* @param[in1]    speed  
* @param[in2]    direction,0-left，1-right
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

	if (direction == 0)   //left
	{
		if (dig_ir_l1 == LOW && dig_ir_r1 == HIGH)
		{
			speed_L1 = speed_L1 - ir_L1;
			speed_L2 = speed_L2 + ir_L1 - 40;
			speed_R1 = speed_R1 + ir_R1 - 40;
			speed_R2 = speed_R2 - ir_R1 + 40;
		}
		else if (dig_ir_l1 == HIGH && dig_ir_r1 == LOW)
		{
			speed_L1 = speed_L1 - ir_L1 + 40;
			speed_L2 = speed_L2 - ir_L1 + 40;
			speed_R1 = speed_R1 + ir_R1 - 40;
			speed_R2 = speed_R2 + ir_R1;
		}
		else if (dig_ir_l1 == LOW && dig_ir_r1 == LOW)
		{
			speed_L1 = speed_L1 - ir_L1 + 43;
			speed_L2 = speed_L2 + ir_L1 - 43;
			speed_R1 = speed_R1 + ir_R1 - 43;
			speed_R2 = speed_R2 - ir_R1 + 43;
		}

		carRun(-speed_L1 + pid_mid, speed_R1 + pid_mid, speed_L2 + pid_mid, -speed_R2 + pid_mid);
	}
	else if (direction == 1) //right
	{
		if (dig_ir_l1 == LOW && dig_ir_r1 == HIGH)
		{
			speed_L1 = speed_L1 + ir_L1;
			speed_L2 = speed_L2 - ir_L1 + 40;
			speed_R1 = speed_R1 - ir_R1 + 40;
			speed_R2 = speed_R2 + ir_R1 - 40;
		}
		else if (dig_ir_l1 == HIGH && dig_ir_r1 == LOW)
		{
			speed_L1 = speed_L1 + ir_L1 - 40;
			speed_L2 = speed_L2 + ir_L1 - 40;
			speed_R1 = speed_R1 - ir_R1 + 40;
			speed_R2 = speed_R2 - ir_R1;
		}
		else if (dig_ir_l1 == LOW && dig_ir_r1 == LOW)
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
* Function       mode_surround
* @author        chengengyue
* @date          2019.10.08
* @brief         Surround Mode
* @param[in1]    speed 
* @param[in2]    direction,0-left，1-right
* @param[out]    void
* @retval        void
* @par History   no
*/
void mode_surround()
{
	ir_rgb(); //If sensor detected the obstacle,the RGB light next to it lights up red.
	if (dig_ir_l1 == LOW && dig_ir_l2 == LOW)
	{
		car_direction = 0;
	}
	else if (dig_ir_r1 == LOW && dig_ir_r2 == LOW)
	{
		car_direction = 1;
	}

	if (car_direction == 1)
	{
		carSurround(CarSpeedControl, 1);
	}
	else
	{
		carSurround(CarSpeedControl, 0);
	}
}

/**
* Function       carLateral
* @author        chengengyue
* @date          2019.10.08
* @brief         control car pan
* @param[in1]    speed  
* @param[in2]    direction，0-left，1-right
* @param[out]    void
* @retval        void
* @par History   no
*/
void carLateral(int speed, int direction)
{
	int speed_L1 = speed;
	int speed_L2 = speed;
	int speed_R1 = speed;
	int speed_R2 = speed;

	ir_L1 = analogRead(IR_SENSOR_L1) / 10;
	ir_L2 = analogRead(IR_SENSOR_L2) / 10;
	ir_R1 = analogRead(IR_SENSOR_R1) / 10;
	ir_R2 = analogRead(IR_SENSOR_R2) / 10;

	int pid_l1 = PIDCal_IR(PID_IR, ir_L1);
	int pid_l2 = PIDCal_IR(PID_IR, ir_L2);
	int pid_r1 = PIDCal_IR(PID_IR, ir_R1);
	int pid_r2 = PIDCal_IR(PID_IR, ir_R2);

	if (direction == 0)  //left
	{
		speed_L1 = speed_L1 + pid_l2;
		speed_L2 = speed_L2 + pid_l1;
		speed_R1 = speed_R1 + pid_r1;
		speed_R2 = speed_R2 + pid_r2;

		carRun(-speed_L1, speed_R1, speed_L2, -speed_R2);
	}
	else if (direction == 1)  //right
	{
		speed_L1 = speed_L1 + pid_l1;
		speed_L2 = speed_L2 + pid_l2;
		speed_R1 = speed_R1 + pid_r2;
		speed_R2 = speed_R2 + pid_r1;

		carRun(speed_L1, -speed_R1, -speed_L2, speed_R2);
	}
}

/**
* Function       mode_lateral
* @author        chengengyue
* @date          2019.10.08
* @brief         left pan mode
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void mode_lateral()
{
	ir_rgb(); //If sensor detected the obstacle,the RGB light next to it lights up red.
	state_direction = 0;
	carLateral(CarSpeedControl, state_direction);
}

/**
* Function       obstacle
* @author        chengengyue
* @date          2019.10.08
* @brief         obstacle
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
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
	else //In other cases, the car keeps going straight.
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

void mode_obstacle()
{
	obstacle();
}

/**
* Function       rocker_carRun
* @author        chengengyue
* @date          2019.12.03
* @brief         rocaker control car
* @param[in1]    x (0~255) Control X direction speed
* @param[in2]    y (0~255) Control Y direction speed
* @param[in3]    spin (-1/0/1)
* @param[out]    void
* @par History   no
*/
void rocker_carRun(int x, int y, int spin, float yaw)
{
	int speed_yaw_b = 0, speed_yaw_f = 0;

	if (abs(y - 128) >= 100 && abs(x - 128) <= 50)
	{
		x = 128;
	}
	else if (abs(x - 128) >= 100 && abs(y - 128) <= 50)
	{
		y = 128;
	}

	if (abs(x - 128) < 30 && abs(y - 128) < 30)
	{
		x = 128;
		y = 128;
		omega_PID.SetPoint = ypr[0];
		if (carDir == 1)
		{
			alpha_PID.SetPoint = ypr[0];
			carDir = 0;
		}
	}
	else if (abs(x - 128) > 30 && abs(y - 128) < 30)
	{
		speed_yaw_f = CarSpeedControl * sin(yaw);
		speed_yaw_b = speed_yaw_f;
		carDir = 1;
	}
	else if (abs(x - 128) < 30 && abs(y - 128) > 30)
	{
		speed_yaw_f = CarSpeedControl * sin(yaw);
		speed_yaw_b = 0;
		carDir = 1;
	}
	else if (abs(x - 128) >= 30 && abs(y - 128) >= 30)
	{
		speed_yaw_f = CarSpeedControl * sin(yaw);
		speed_yaw_b = speed_yaw_f;
		carDir = 1;
	}
	else
	{
		carDir = 1;
	}

	if (spin == 0)
	{
		if (carDir == 1)
		{
			alpha_PID.SetPoint = ypr[0];
			carDir = 0;
		}
	}
	else
	{
		carDir = 1;
	}

	int speed_X = map(x, 0, 255, -CarSpeedControl, CarSpeedControl);
	int speed_Y = map(y, 0, 255, -CarSpeedControl, CarSpeedControl);
	int spin_A = map(spin, -1, 1, -30, 30);

	int speed_L1 = speed_Y + speed_X + spin_A - speed_yaw_f;
	int speed_L2 = speed_Y - speed_X + spin_A - speed_yaw_b;
	int speed_R1 = speed_Y - speed_X - spin_A + speed_yaw_f;
	int speed_R2 = speed_Y + speed_X - spin_A + speed_yaw_b;

	carRun(speed_L1, speed_R1, speed_L2, speed_R2);
	// delay(20);
}

/**
* Function       carRun
* @author        chengengyue
* @date          2019.10.08
* @brief         Enter four motor speed values to control trolley operation
* @param[in1]    speed_L1 （-160~160）
* @param[in2]    speed_R1 （-160~160）
* @param[in3]    speed_L2 （-160~160）
* @param[in4]    speed_R2 （-160~160）
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
* Function       PIDCal_car
* @author        chengengyue
* @date          2019.10.08
* @brief         PID Calculate the yaw angle
* @param[in]     NextPoint
* @param[out]    -omega_rad
* @retval        void
* @par History   no
*/
float PIDCal_car(float NextPoint)
{
	float dError, Error;
	Error = omega_PID.SetPoint - NextPoint;				//Deviation
	omega_PID.SumError += Error;						//Integral
	dError = omega_PID.LastError - omega_PID.PrevError; //Current differential
	omega_PID.PrevError = omega_PID.LastError;
	omega_PID.LastError = Error;

	double omega_rad = omega_PID.Proportion * Error				 //Proportional term
					   + omega_PID.Integral * omega_PID.SumError //Integral term
					   + omega_PID.Derivative * dError;			 //Differential term

	if (omega_rad > M_PI / 6)
		omega_rad = M_PI / 6;
	if (omega_rad < -M_PI / 6)
		omega_rad = -M_PI / 6;
	return -omega_rad;
}

/**
* Function       PIDCal_IR
* @author        chengengyue
* @date          2019.10.08
* @brief         PID Calculate the value of the sensor
* @param[in1]    pid
* @param[in2]    nowValue
* @param[out]    -val_sensor
* @retval        void
* @par History   no
*/
float PIDCal_IR(PID pid, float nowValue)
{
	float dError, Error;
	Error = pid.SetPoint - nowValue;		//deviation
	pid.SumError += Error;					//Integral
	dError = pid.LastError - pid.PrevError; //Current differential
	pid.PrevError = pid.LastError;
	pid.LastError = Error;

	double val_sensor = pid.Proportion * Error		  //Proportional term
						+ pid.Integral * pid.SumError //Integral term
						+ pid.Derivative * dError;	//Differential term
	return -val_sensor;
}

/**
* Function       PIDCal_Stabilize
* @author        chengengyue
* @date          2019.12.03
* @brief         PIDCal_Stabilize
* @param[in1]    pid
* @param[in2]    nowValue
* @param[out]    void
* @retval        void
* @par History   no
*/
float PIDCal_Stabilize(PID pid, float nowValue)
{
	float dError, Error;
	Error = pid.SetPoint - nowValue; //Deviation
  //The following is a special case: the initial angle of the gyroscope is zero, it turns to the left as a negative value, and it turns to the right as a positive value, which coincides at 180 degrees and -180 degrees
  //If the following treatments are not added, it will cause the car to stop by 180 degrees. If the car is touched manually, the car will always rotate without stopping.
	if (pid.SetPoint < -(M_PI * 5 / 6) || pid.SetPoint > (M_PI * 5 / 6))
	{
		if (pid.SetPoint < 0) // -180 < X < -150
		{
			if ((M_PI + pid.SetPoint) < nowValue || pid.SetPoint > nowValue) //Clockwise
			{
				if (nowValue < 0) // -180 < Y < X
				{
					Error = 2 * M_PI + pid.SetPoint + nowValue;
				}
				else // Y > pi-X
				{
					Error = 2 * M_PI + pid.SetPoint - nowValue;
				}
			}
			else if ((M_PI + pid.SetPoint) > nowValue && pid.SetPoint < nowValue) //Counterclockwise
			{
				Error = pid.SetPoint - nowValue;
			}
		}
		else // 150 < X < 180
		{
			if (nowValue < (-M_PI + pid.SetPoint) || pid.SetPoint < nowValue) //Counterclockwise
			{
				if (pid.SetPoint < nowValue) 
				{
					Error = -(nowValue - pid.SetPoint);
				}
				else if (abs(pid.SetPoint) > abs(nowValue)) 
				{
					Error = -(2 * M_PI - pid.SetPoint + nowValue);
				}
				else 
				{
					Error = -(pid.SetPoint + nowValue);
				}
			}
			else if ((-M_PI + pid.SetPoint) < nowValue && pid.SetPoint > nowValue) //Clockwise
			{
				Error = pid.SetPoint - nowValue;
			}
		}
	}
	else
	{
		Error = pid.SetPoint - nowValue; //Deviation
	}

	pid.SumError += Error;					//Integral
	dError = pid.LastError - pid.PrevError; //Current differential
	pid.PrevError = pid.LastError;
	pid.LastError = Error;

	double pid_value = pid.Proportion * Error		 
					   + pid.Integral * pid.SumError 
					   + pid.Derivative * dError;	

	if (pid_value > M_PI / 6)
		pid_value = M_PI / 6;
	if (pid_value < -M_PI / 6)
		pid_value = -M_PI / 6;
	return -pid_value;
}

/**
* Function       Radians_To_Degrees
* @author        chengengyue
* @date          2019.12.03
* @brief         Radian-> Angle Degree
* @param[in]     a
* @param[out]    void
* @retval        void
* @par History   no
*/
float Radians_To_Degrees(float a)
{
	return (a * 180 / M_PI); //angle
}

/**
* Function       servo
* @author        chengengyue
* @date          2019.10.08
* @brief         control servo angle
* @param[in1]    pid
* @param[in2]    degree
* @param[out]    void
* @retval        void
* @par History   no
*/
void servo(int degree)
{
	long us = (degree * 1800 / 180 + 600); // 0.6 ~ 2.4
	long pwmvalue = us * 4096 / 20000;	 // 50hz: 20,000 us
	pwm.setPWM(7, 0, pwmvalue);
}

/**
* Function       keyscan
* @author        chengengyue
* @date          2019.12.03
* @brief         keyscan
* @param[in1]    void
* @retval        void
* @par History   no
*/
void keyscan()
{
	int val;
	val = digitalRead(KEY_PIN); 
	if (val == LOW)				
	{
		delay(10);					
		val = digitalRead(KEY_PIN); 
		while (val == LOW)
		{
			val = digitalRead(KEY_PIN); 
			if (val == HIGH)			
			{
				whistle();
				alpha_PID.SetPoint = ypr[0];
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
* Function       serial_data_parse
* @author        chengengyue
* @date          2019.10.08
* @brief         Analyze the general protocol instructions sent by the host computer and perform the corresponding actions
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void serial_data_parse()
{
	if (InputString.indexOf("Mode") >= 0)
	{
		whistle();
		if (InputString[6] == '0' && InputString[7] == '0')
		{
			showRGB(MAX_LED, 0, 255, 0);
			g_modeSelect = 0;
		}
		else if (InputString[6] == '1' && InputString[7] == '1')
		{
			g_modeSelect = 1;
		}
		else if (InputString[6] == '2' && InputString[7] == '1')
		{
			g_modeSelect = 2;
		}
		else if (InputString[6] == '3' && InputString[7] == '1')
		{
			g_modeSelect = 3;
		}
	}
	//No-apk mode exit
	if (g_modeSelect != 0)
	{
		InputString = ""; //clear serial port data
		NewLineReceived = false;
		return;
	}

  //Analyze the general protocol instructions sent by the host computer and perform the corresponding actions
  //Eg:$Music,11#    play music
	if (InputString.indexOf("Servo") >= 0)
	{
		servo_degree = InputString.substring(7, 10).toInt();
		servo_degree = map(servo_degree, 0, 180, 150, 40);
		servo(servo_degree);
	}
	else if (InputString.indexOf("Music") >= 0)
	{
		if (InputString[7] == '0' && InputString[8] == '0')
		{
			buzzer_music = 0;
			music_index = 0;
		}
		else if (InputString[8] == '1')
		{
			music_index = 0;
			switch (InputString[7])
			{
			case '1':
				buzzer_music = 1;
				break;
			case '2':
				buzzer_music = 2;
				break;
			case '3':
				buzzer_music = 3;
				break;
			case '4':
				buzzer_music = 4;
				break;
			case '5':
				buzzer_music = 5;
				break;
			default:
				break;
			}
		}
	}
	else if (InputString.indexOf("Buzzer") >= 0)  //whistle
	{
		if (InputString[8] == '1' && InputString[9] == '1')
		{
			buzzer_state = 1;
		}
	}
	else if (InputString.indexOf("RGB") >= 0) //RGB effect
	{
		whistle();
		if (InputString[5] == '0' && InputString[6] == '0')
		{
			RGB_Effect = '5';
		}
		else if (InputString[6] == '1')
		{
			RGB_Effect = InputString[5];
		}
	}
	else if (InputString.indexOf("Speed") >= 0)
	{
		if (InputString[8] == '1')
		{
			if (InputString[7] == '1') // Accelerate, add 10 each time
			{
				CarSpeedControl += 10;
				if (CarSpeedControl >= 60)
					CarSpeedControl = 60;
			}
			else if (InputString[7] == '2') //Slow down, minus 10 each time
			{
				CarSpeedControl -= 10;
				if (CarSpeedControl <= 30)
					CarSpeedControl = 30;
			}
		}
	}

	if (InputString.indexOf("X") >= 0 && InputString.indexOf("Y") >= 0)
	{
		Rocker_X = InputString.substring(2, 5).toInt();
		Rocker_Y = InputString.substring(7, 10).toInt();
		if (Rocker_X == 128 && Rocker_Y == 128)
		{
			spin = 0;
		}
	}

	if (InputString.indexOf("Spin") >= 0)
	{
		if (InputString[6] == '0' && InputString[7] == '0')
		{
			spin = 0;
		}
		else if (InputString[7] == '1')
		{
			if (InputString[6] == '1') //spin left
			{
				spin = -1;
			}
			else if (InputString[6] == '2') //spin right
			{
				spin = 1;
			}
		}
	}

	InputString = ""; //Clear serial port
	NewLineReceived = false;
}

/**
* Function       serialEvent
* @author        chengengyue
* @date          2019.10.08
* @brief         serialEvent
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void serialEvent()
{
	while (Serial.available())
	{
		//One byte is read one byte, and the next sentence is read into the string array to form a completed packet.
		IncomingByte = Serial.read();
		if (IncomingByte == '$')
		{
			StartBit = true;
		}
		if (StartBit == true)
		{
			InputString += (char)IncomingByte;
			Receive_Length++;
			if (Receive_Length >= 15)
			{
				StartBit = false;
				Receive_Length = 0;
				InputString = "";
			}
		}
		if (StartBit == true && IncomingByte == '#')
		{
			StartBit = false;
			Receive_Length = 0;
			int len = InputString.length();
			if (len >= 8 && len <= 13)
			{
				NewLineReceived = true;
			}
			else
			{
				NewLineReceived = false;
				InputString = "";
			}
		}
	}
}

/**
* Function       setup
* @author        chengengyue
* @date          2019.12.03
* @brief         Initialization setting
* @param[in]     void
* @retval        void
* @par History   no
*/
void setup()
{
	pinMode(KEY_PIN, INPUT_PULLUP); 
	pinMode(INTERRUPT_PIN, INPUT);  
	pinMode(LED_PIN, OUTPUT);		
	pinMode(BUZZER, OUTPUT);		
	pinMode(IR_SENSOR_L1, INPUT);   
	pinMode(IR_SENSOR_L2, INPUT);
	pinMode(IR_SENSOR_R1, INPUT);
	pinMode(IR_SENSOR_R2, INPUT);
	pinMode(IR_SENSOR_MID, INPUT);

	//Serial port baud rate setting
	Serial.begin(9600);

	//Close RGB light initialization
	strip.begin();
	strip.show();
	clearRGB();

	pwm.begin();
	pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
	brake();			//Car stop

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	digitalWrite(LED_PIN, HIGH); //Turn off the gyroscope status indicator
	initMPU6050();				 //MPU6050 Initialization

	delay(500);
	boot_animation();
}

/**
* Function       loop
* @author        chengengyue
* @date          2019.12.03
* @brief         main function
* @param[in]     void
* @retval        void
* @par History   no
*/
void loop()
{
	serialEvent();
	if (NewLineReceived)
	{
		serial_data_parse(); //Call the serial port analytic function
	}

	if (g_modeSelect != 0)
	{
		switch (g_modeSelect)
		{
		case 1:
			mode_surround();
			break;
		case 2:
			mode_obstacle();
			break;
		case 3:
			mode_lateral();
			break;
		default:
			break;
		}
	}
	else if (buzzer_music != 0)
	{
		music_play(buzzer_music, music_index);
		music_index++;
		if (music_index >= music_max[buzzer_music - 1])
		{
			buzzer_music = 0;
			music_index = 0;
		}
	}
	else if (RGB_Effect != 0)
	{
		switch (RGB_Effect)
		{
		case '1': //breathing light
			nowTime = millis();
			if (nowTime - oldTime >= 20)
			{
				oldTime = nowTime;
				breathing_RGB();
			}
			break;
		case '2': //Marquee
			nowTime = millis();
			if (nowTime - oldTime >= 150)
			{
				oldTime = nowTime;
				marquee_RGB();
			}
			break;
		case '3': //water light
			nowTime = millis();
			if (nowTime - oldTime >= 300)
			{
				oldTime = nowTime;
				waterfall_RGB();
			}
			break;
		case '4': //Colorful RGB light
			nowTime = millis();
			if (nowTime - oldTime >= 10)
			{
				oldTime = nowTime;
				colorfull_RGB();
			}
			break;
		case '5': //Close RGB light
			showRGB(MAX_LED, 0, 0, 0);
			RGB_Effect = 0;
			break;

		default:
			break;
		}
	}
	else if (buzzer_state)
	{
		whistle();
		buzzer_state = 0;
	}
	else
	{
		mpu6050_getdata();
		omega_Work = PIDCal_car(ypr[0]);
		alpha_Work = PIDCal_Stabilize(alpha_PID, ypr[0]);

		//According to the state of the car, do the corresponding action
		rocker_carRun(Rocker_X, Rocker_Y, spin, omega_Work);

		//Car self-stabilizing function adjustment
		if (carDir == 0)
		{
			int nowDegree = abs(Radians_To_Degrees(ypr[0]));
			int point = abs(Radians_To_Degrees(alpha_PID.SetPoint));
			int offset = abs(nowDegree - point);
			int speed_spin = 0;

			if (offset >= 3)
			{
				speed_spin = 50 * sin(alpha_Work);
				carRun(-speed_spin, speed_spin, -speed_spin, speed_spin);
			}
		}
	}
}
