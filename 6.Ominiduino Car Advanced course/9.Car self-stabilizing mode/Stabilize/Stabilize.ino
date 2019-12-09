/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Stabilize.ino
* @author       chengengyue
* @version      V1.1
* @date         2019.12.03
* @brief        Car self-stabilizing mode
* @details
* @par History  no
*
*/
//Import library file
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define ENABLE_DEBUG1

#define KEY_PIN 8		//Define button pins
#define INTERRUPT_PIN 2 //Define MPU6050 pins
#define LED_PIN 5		
#define RGB_PIN 9		//Define RGB pins
#define MAX_LED 4		//Car with 4 RGB lights
#define BUZZER 10		//Define buzzer pins

//MPU6050、pca9685、RGB
MPU6050 mpu = MPU6050(0x68);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_LED, RGB_PIN, NEO_RGB + NEO_KHZ800);

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

const char wheel[4][2] = {{10, 11}, {13, 12}, {15, 14}, {8, 9}};
float wheel_speed[4];
int CarSpeedControl = 40;

int carDir = 0; //Car self-stabilizing adjustment flag, 0 is adjustment (stop), 1 is non-adjustment (remote control)

/*====================================================================================================
PID Function
The PID (Proportional, integral, differential) function is used in mainly
CarSpeedControl applications. PIDCalc performs one iteration of the PID
algorithm.
While the PID function works, main is just a dummy program showing
a typical usage.
=====================================================================================================*/
typedef struct
{
	float SetPoint;   //Desired value
	float Proportion; //Proportional Const
	float Integral;   //Integral Const
	float Derivative; //Derivative Const
	float LastError;  // Error[-1]
	float PrevError;  // Error[-2]
	float SumError;   // Sums of Errors
} PID;

float PIDCal_Stabilize(PID pid, float nowValue);

/*====================================================================================================/
PID Calculation section
=====================================================================================================*/
PID alpha_PID = {0, 5, 0, 0.1, 0, 0, 0};
float alpha_Work = 0;

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
* Function       clearRGB
* @author        chengengyue
* @date          2019.10.08
* @brief         set onboard RGB off
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
* Function       carRun
* @author        chengengyue
* @date          2019.10.08
* @brief         Enter four motor speed values to control trolley operation
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
	if (pid.SetPoint < -(M_PI * 5/6) || pid.SetPoint > (M_PI * 5/6))
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
		Error = pid.SetPoint - nowValue;		//Deviation
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
* Function       setup
* @author        chengengyue
* @date          2019.12.03
* @brief         
* @param[in]     void
* @retval        void
* @par History  no
*/
void setup()
{
  pinMode(KEY_PIN, INPUT_PULLUP); 
	pinMode(INTERRUPT_PIN, INPUT); 
	pinMode(LED_PIN, OUTPUT);	  
	pinMode(BUZZER, OUTPUT);	   

	//Serial port baud rate setting
	Serial.begin(9600);

	//Close RGB light initialization
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

	digitalWrite(LED_PIN, HIGH); //Turn off the gyroscope status indicator
	initMPU6050();				//MPU6050 Initialization

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
	keyscan();
	mpu6050_getdata();
	// Serial.println(ypr[0] * 180 / M_PI);
	alpha_Work = PIDCal_Stabilize(alpha_PID, ypr[0]);

	int nowDegree = abs(Radians_To_Degrees(ypr[0]));
	int point = abs(Radians_To_Degrees(alpha_PID.SetPoint));
	int offset = abs(nowDegree - point);

	if (offset >= 3)
	{
		int speed_spin = 50 * sin(alpha_Work);
		carRun(-speed_spin, speed_spin, -speed_spin, speed_spin);
	}
	else
	{
		carRun(0, 0, 0, 0);
	}
}
