/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         MPU6050_DMP.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        MPU6050_DMP
* @details
* @par History  no
*/
//Import library
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include "Wire.h"

#define KEY_PIN 8       //Define Key pin
#define INTERRUPT_PIN 2 //Define the MPU6050 interrupt pin
#define LED_PIN 5       //Define gyroscope status light pins

//Initialize PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
//Initialize MPU6050
MPU6050 mpu = MPU6050(0x68);

int CarSpeedControl = 60; //Car speed
int g_CarState  = 0;      //Car status
int Rocker_X = 128;       //Analog rocker X value
int Rocker_Y = 128;       //Analog rocker Y value
int spin = 0;             //Direction of rotation, -1 is spin left, 1 is spin right and 0 is stop.

const char wheel[4][2] = {{10, 11}, {13, 12}, {15, 14}, {8, 9}};

/*Car running status enumeration*/
const typedef enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enSPINLEFT,
  enSPINRIGHT,
  enFRONTLEFT,
  enFRONTRIGHT,
  enREARLEFT,
  enREARRIGHT
} enCarState;

//Key status
bool button_press = false;

/*Servo related parameters*/
int servo_degree = 90;

/*Serial data setting*/
int IncomingByte = 0;            //Received data byte
int Receive_Length = 0;          //Length of data
String InputString = "";         //Used to store received content
boolean NewLineReceived = false; //Previous data end mark
boolean StartBit = false;        //Agreement start sign

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

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
  float Proportion; //Proportional Const
  float Integral;   //Integral Const
  float Derivative; //Derivative Const
  float LastError;  //Error[-1]
  float PrevError;  //Error[-2]
  float SumError;   //Sums of Errors
} PID;

/*PID calculation section*/
PID omega_PID = {0, 0.4, 0, 0.1, 0, 0, 0};

float omega_Work = 0;

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
  pwm.setPWM(10, 0, Speed);    //Right front wheel forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);     //Right rear wheel forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed);    //Left front wheel forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed);    //Left rear wheel forward
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
  pwm.setPWM(11, 0, Speed);    //Right front wheel reserve
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);     //Right rear wheel reserve

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed);    //Left front wheel reserve
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);    //Left rear wheel reserve
}

/**
* Function       left
* @author        chengengyue
* @date          2019.10.08
* @brief         car left translation(A type wheel Reverse，B type wheel Forward)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   NO
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
* @par History   NO
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
* @par History   NO
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
* @par History   NO
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
* Function       front_left
* @author        chengengyue
* @date          2019.10.08
* @brief         car moves to the left front (A type wheel stop，B type wheel Forward)
* @param[in]     Speed(0-160)
* @param[out]    void
* @retval        void
* @par History   NO
*/
void front_left(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed);    //Right front wheel(B type) Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, 0);         //Right rear wheel(A type) stop
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, 0);        //Left front wheel(A type) stop
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed);    //Left rear wheel(B type) Forward
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
* @par History   NO
*/
void front_right(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);        //Right front wheel(B type) stop
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);     //Right rear wheel(A type) Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed);    //Right rear wheel(A type) Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, 0);        //Left rear wheel(B type) stop
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
* @par History   NO
*/
void left_rear(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);        //Right front wheel(B type) stop
  pwm.setPWM(11, 0, 0);    
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);     //Right rear wheel(A type) Reserve

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed);    //Left front wheel(A type) Reserve
  pwm.setPWM(15, 0, 0);        //Left rear wheel(B type) stop
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
* @par History   NO
*/
void right_rear(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, Speed);    //Right front wheel(B type) Reserve
  pwm.setPWM(8, 0, 0);         //Right rear wheel(A type) stop
  pwm.setPWM(9, 0, 0);     

  pwm.setPWM(13, 0, 0);        //Left front wheel (A type) stop
  pwm.setPWM(12, 0, 0);    
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed);    //Left rear wheel(B type) Reserve
}

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
* Function       serialEvent
* @author        chengengyue
* @date          2019.10.08
* @brief         Serial port unpacking
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
    if (IncomingByte == '$')        // $Indicates that the agreement begins
    {
      StartBit = true;
    }
    if (StartBit == true)          // Start receiving content
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
    // #indicates the end of the agreement
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
  // Extract servo angle values and limit angles
  if (InputString.indexOf("Servo") >= 0)
  {
    //Extract servo angle values and limit angles
    servo_degree = InputString.substring(7, 10).toInt();
    servo_degree = map(servo_degree, 0, 180, 150, 30);
    servo(servo_degree);  //set servo angle
  }
  
  //Parsing the APP ADD, DECE button protocol(eg:Accelerate):$Speed,11#)
  if (InputString.indexOf("Speed") >= 0)
  {
    if (InputString[8] == '1')
    {
      if (InputString[7] == '1')  //Accelerate, add 20 each time
      {
        CarSpeedControl += 20;
        if (CarSpeedControl >= 160)
          CarSpeedControl = 160;
      }
      else if (InputString[7] == '2')  //Slow down, minus 20 each time
      {
        CarSpeedControl -= 20;
        if (CarSpeedControl <= 40)
          CarSpeedControl = 40;
      }
    }
  }

  if (InputString.indexOf("X") >= 0 && InputString.indexOf("Y") >= 0)
  {
    //Extract the data sent by the APP simulation joystick(eg:stop):$X128,Y128#）
    Rocker_X = InputString.substring(2, 5).toInt();
    Rocker_Y = InputString.substring(7, 10).toInt();
    if (Rocker_X == 128 && Rocker_Y == 128)
    {
      spin = 0;
    }

  }

  //Parsing the APP rotation button protocol (example:spin left):$Spin,11#）
  if (InputString.indexOf("Spin") >= 0)
  {
    if (InputString[6] == '0' && InputString[7] == '0')
    {
      spin = 0;
    }
    else if (InputString[7] == '1')
    {
      if (InputString[6] == '1')      //spin left
      {
        spin = -1;
      }
      else if (InputString[6] == '2') //spin right
      {
        spin = 1;
      }
    }
  }
  
  InputString = ""; //clear serial data
  NewLineReceived = false;
}

/**
* Function       initMPU6050
* @author        chengengyue
* @date          2019.10.08
* @brief         MPU6050 initialization
* @param[in1]    void
* @param[out]    void
* @par History   
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

    mpu.setDMPEnabled(true);

// enable Arduino interrupt detection
#ifdef ENABLE_DEBUG
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
#endif

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

// set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef ENABLE_DEBUG
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    digitalWrite(LED_PIN, LOW);      // LED D9 is on
  }
  else
  {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
#ifdef ENABLE_DEBUG
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif

    digitalWrite(LED_PIN, HIGH);     //LED D9 is off
  }
}

/**
* Function       mpu6050_getdata
* @author        chengengyue
* @date          2019.10.08
* @brief         getdata
* @param[in1]    void
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

    digitalWrite(LED_PIN, LOW); //turn on LED9
  }
}

/**
* Function       PIDCal_car
* @author        chengengyue
* @date          2019.10.08
* @brief         PID Calculate the yaw angle
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   no
*/
float PIDCal_car(float NextPoint)
{
  float dError, Error;
  Error = omega_PID.SetPoint - NextPoint;       
  omega_PID.SumError += Error;           
  dError = omega_PID.LastError - omega_PID.PrevError; 
  omega_PID.PrevError = omega_PID.LastError;
  omega_PID.LastError = Error;

  double omega_rad = omega_PID.Proportion * Error        
             + omega_PID.Integral * omega_PID.SumError 
             + omega_PID.Derivative * dError;      

  if (omega_rad > M_PI / 6)
    omega_rad = M_PI / 6;
  if (omega_rad < -M_PI / 6)
    omega_rad = -M_PI / 6;
  return -omega_rad;
}

/**
* Function       carRun
* @author        chengengyue
* @date          2019.10.08
* @brief         Enter four motor speed values to control car operation
* @param[in1]    speed_L1 （-160~160）
* @param[in2]    speed_R1 （-160~160）
* @param[in3]    speed_L2 （-160~160）
* @param[in4]    speed_R2 （-160~160）
* @param[out]    void
* @par History   NO
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
* Function       rocker_carRun
* @author        chengengyue
* @date          2019.10.08
* @brief         rocker control car
* @param[in1]    x (0~255) Control X direction speed
* @param[in2]    y (0~255) Control Y direction speed
* @param[in3]    spin control rotation（-1spin left，1spin right，0 stop）
* @param[out]    void
* @par History   no
*/
void rocker_carRun(int x, int y, int spin, float yaw)
{
  int speed_yaw_b = 0, speed_yaw_f = 0;
  //Debounce
  if (y >= 200 && abs(x - 128) <= 50)
  {
    x = 128;
  }
  else if (x >= 200 && abs(y - 128) <= 50)
  {
    y = 128;
  }

  if (abs(x - 128) < 30 && abs(y - 128) < 30)  
  {
    x = 128;
    y = 128;
    omega_PID.SetPoint = ypr[0];    //Set initial yaw angle
  }
  //Adjust the speed according to the angle
  else if (abs(x - 128) > 30 && abs(y - 128) < 30)
  {
    speed_yaw_f = CarSpeedControl * sin(yaw);
    speed_yaw_b = speed_yaw_f;
  }
  else if (abs(x - 128) < 30 && abs(y - 128) > 30)
  {
    speed_yaw_f = CarSpeedControl * sin(yaw);
    speed_yaw_b = 0;
  }
  

  int speed_X = map(x, 0, 255, -CarSpeedControl, CarSpeedControl);
  int speed_Y = map(y, 0, 255, -CarSpeedControl, CarSpeedControl);
  int spin_A = map(spin, -1, 1, -30, 30);   //Fixed rotation speed of 30

  //Motor speed
  int speed_L1 = speed_Y + speed_X + spin_A - speed_yaw_f;
  int speed_L2 = speed_Y - speed_X + spin_A - speed_yaw_b;
  int speed_R1 = speed_Y - speed_X - spin_A + speed_yaw_f;
  int speed_R2 = speed_Y + speed_X - spin_A + speed_yaw_b;

  //Drive car
  carRun(speed_L1, speed_R1, speed_L2, speed_R2);
  delay(20);
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
  pinMode(KEY_PIN, INPUT_PULLUP); 
  pinMode(INTERRUPT_PIN, INPUT);  
  pinMode(LED_PIN, OUTPUT);      
  Serial.begin(9600);
  Wire.begin();       
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  brake();            

  digitalWrite(LED_PIN, HIGH); 
  initMPU6050();         
  delay(1000);
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
  serialEvent();        //Receive serial port information and save protocol content
  if (NewLineReceived)
  {
    serial_data_parse(); //Call the serial port analytic function
  }
  //Get gyroscope data and refresh the current yaw angle
  mpu6050_getdata();
  omega_Work = PIDCal_car(ypr[0]);

  //Control the car according to the instructions sent by the APP
  rocker_carRun(Rocker_X, Rocker_Y, spin, omega_Work);

}
