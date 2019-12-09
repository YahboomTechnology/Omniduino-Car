/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         WIFI_Control.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        WiFi camera control car
* @details
* @par History  no
*/
//Import library
#include <Adafruit_PWMServoDriver.h>
#include "Wire.h"

#define KEY_PIN 8    //Define k1 button pins

//PCA9685 Initialization
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//Car control parameters
int CarSpeedControl = 60;
int g_CarState  = 0;

/*Car running status enumeration */
const typedef enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enSPINLEFT,
  enSPINRIGHT
} enCarState;

//Button status
bool button_press = false;

/*serial data setting*/
int IncomingByte = 0;            //Received data byte
int Receive_Length = 0;          //Receive data length 
String InputString = "";         //Used to store received content
boolean NewLineReceived = false; //Previous data end mark
boolean StartBit = false;        //Agreement start sign


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
  pwm.setPWM(10, 0, Speed);    //Right front wheel Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);     //Right rear wheel Forward
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
  pwm.setPWM(14, 0, Speed);    //Left rear reserve
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
* @par History   no
*/
void front_right(int Speed)
{
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);        //Right front wheel(B type) stop
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);     //Right rear wheel(A type) Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed);    //Left front wheel(A type) Forward
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
* @par History   no
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
* @par History   no
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
* Function       carDrifting
* @author        chengengyue
* @date          2019.10.08
* @brief         carDrifting
* @param[in1]    Speed_Front(0-160)
* @param[in2]    Speed_Rear(0-160)
* @param[out]    void
* @retval        void
* @par History   no
*/
void carDrifting(int Speed_Front, int Speed_Rear)
{
  Speed_Front = map(Speed_Front, 0, 160, 0, 2560);
  Speed_Rear = map(Speed_Rear, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed_Front);   //Right front forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed_Rear);     //Right rear reserve

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed_Front);   //Left front reserve
  pwm.setPWM(15, 0, Speed_Rear);    //Left rear forward
  pwm.setPWM(14, 0, 0);
}

/**
* Function       carRun01
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     Speed
* @param[out]    void
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
* Function       carRun02
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   no
*/
void carRun02()
{
  front_left(CarSpeedControl);     
  delay(1000);
  front_right(CarSpeedControl);    
  delay(1000);
  right_rear(CarSpeedControl);     
  delay(1000);
  left_rear(CarSpeedControl);      
  delay(1000);
  brake();                          
}

/**
* Function       carRun03
* @author        chengengyue
* @date          2019.10.08
* @brief         
* @param[in]     Speed
* @param[out]    void
* @retval        void
* @par History   no
*/
void carRun03()
{
  //Reduce the front wheel speed so that the rear wheel speed is larger than the front wheel speed
  carDrifting(CarSpeedControl*0.6, CarSpeedControl);
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
    if (IncomingByte == '$')        //$Indicates that the agreement begins
    {
      StartBit = true;
    }
    if (StartBit == true)          //Start receiving content
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
  //Car advance,back,left translation,right translation,stop
  if (InputString.indexOf("X") >= 0 && InputString.indexOf("Y") >= 0)
  {
    //Extract the data sent by the APP simulation joystick(eg:stop):$X128,Y128#）
    int X = InputString.substring(2, 5).toInt();
    int Y = InputString.substring(7, 10).toInt();
    if (X == 128 && Y == 128)  // stop
    {
      g_CarState = enSTOP;
    }
    else if (X <= 5)           //Left translation
    {
      g_CarState = enLEFT;
    }
    else if (X >= 255)         //Right translation
    {
      g_CarState = enRIGHT;
    }
    else if (Y <= 5)           //Back
    {
      g_CarState = enBACK;
    }
    else if (Y >= 250)         //Adavance
    {
      g_CarState = enRUN;
    }
  }

  //Parsing the APP rotation button protocol(eg:spin left):$Spin, 11#)
  if (InputString.indexOf("Spin") >= 0)
  {
    if (InputString[6] == '0' && InputString[7] == '0')
    {
      g_CarState = enSTOP;
    }
    else if (InputString[7] == '1')
    {
      if (InputString[6] == '1')      //Spin left
      {
        g_CarState = enSPINLEFT;
      }
      else if (InputString[6] == '2') //Spin right
      {
        g_CarState = enSPINRIGHT;
      }
    }
  }
  
  InputString = ""; //clear serial port data
  NewLineReceived = false;
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
  Serial.begin(9600);
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
  serialEvent();        //Receive serial port information and save protocol content
  if (NewLineReceived)
  {
    serial_data_parse(); //Call the serial port analytic function
  }

  //According to the state of the car, do the corresponding action
  switch (g_CarState)
  {
  case enSTOP:
    brake();
    break;
  case enRUN:     //Advance
    run(CarSpeedControl);
    break;
  case enLEFT:    //Left translation
    left(CarSpeedControl);
    break;
  case enRIGHT:   //Right translation
    right(CarSpeedControl);
    break;
  case enBACK:    //back
    back(CarSpeedControl);
    break;
  case enSPINLEFT://spin left
    spin_left(CarSpeedControl);
    break;
  case enSPINRIGHT://spin right
    spin_right(CarSpeedControl);
    break;

  default:
    brake();
    break;
  }
}