/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         RGB.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Button start RGB
* @details
* @par History  NO
*/
//Import library file
#include <Adafruit_NeoPixel.h>

#define BUZZER 10    //Define buzzer pin
#define KEY_PIN 8    //Define key pin
#define LED_PIN 5    //Define LED(D9) pin
#define RGB_PIN 9    //Define RGB pin
#define MAX_RGB 4    //4 RGB lights

//Initialize the WS2812 programming light
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_RGB, RGB_PIN, NEO_RGB + NEO_KHZ800);
//Button status
bool button_press = false;

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
* Function       clearRGB
* @author        chengengyue
* @date          2019.10.08
* @brief         set on board RGB light
* @param[in1]    void
* @param[out]    void
* @retval        void
* @par History   no
*/
void clearRGB()
{
  uint32_t color = strip.Color(0, 0, 0);
  for (uint8_t i = 0; i < MAX_RGB; i++)
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
* @param[in1]    num  Which lamp to choose, 
*                if it is greater than or equal to the maximum value, it will be fully lit
* @param[in2]    R value(0~255)
* @param[in3]    G value(0~255)
* @param[in4]    B value(0~255)
* @param[out]    void
* @retval        void
* @par History   no
*/
void showRGB(int num, int R, int G, int B)
{
  uint32_t color = strip.Color(G, R, B);
  if (num >= MAX_RGB)    
  {
    for (int i = 0; i < MAX_RGB; i++)
    {
      strip.setPixelColor(i, color);
    }
  }
  else                   
  {
    strip.setPixelColor(num, color);
  }
  strip.show();
}

/**
* Function       setup
* @author        chengengyue
* @date          2019.10.08
* @brief         Initialization settings
* @param[in]     void
* @param[out]    void
* @retval        void
* @par History   no
*/
void setup() {
  // put your setup code here, to run once:
  pinMode(KEY_PIN, INPUT_PULLUP);//Set the button pin to pull-up input mode
  pinMode(LED_PIN, OUTPUT);      //Set the LED pin to output mode
  pinMode(BUZZER, OUTPUT);       //Define buzzer pin
  pinMode(RGB_PIN, OUTPUT);      //Set the RGB pin to output mode
  
  strip.begin();
  strip.show();
  clearRGB();                      
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
void loop() {
  //put your main code here, to run repeatedly:
  //keyscan
  keyscan();

  if (button_press)
  {
    whistle();                      
    showRGB(MAX_RGB, 255, 0, 0);     //All RGB light become red 0.5s
    delay(500);
    showRGB(MAX_RGB, 0, 255, 0);     //All RGB light become green 0.5s
    delay(500);
    showRGB(MAX_RGB, 0, 0, 255);     //All RGB light become blue 0.5s
    delay(500);
    showRGB(MAX_RGB, 255, 255, 255); //All RGB light become whitle 0.5s
    delay(500);
    button_press = false;            
  }
  else 
  {
    clearRGB();                      
  }
}
