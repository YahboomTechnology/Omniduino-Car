/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         RGBEffects.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        RGBEffects
* @details
* @par History  no
*/
//Import library
#include <Adafruit_NeoPixel.h>

#define KEY_PIN 8    //Define Key pin
#define RGB_PIN 9    //Define the RGB pin
#define MAX_LED 4    //4 RGB lights

//Initialize WS2812 programming RGB
Adafruit_NeoPixel strip = Adafruit_NeoPixel(MAX_LED, RGB_PIN, NEO_RGB + NEO_KHZ800);

/*RGBEffects*/
int RGB_Effect = 0;     //1:Breathing light,2:Marquee,3:Water light,4:Color light,5:Turn off light

//Button status
bool button_press = false;

unsigned long oldTime = 0;
unsigned long nowTime = 0;


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
        RGB_Effect++;
        return;
      }
    }
  }
}

/**
* Function       clearRGB
* @author        chengengyue
* @date          2019.10.08
* @brief         set On board RGB
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
* @param[in1]    num  Which lamp to choose, 
*                if it is greater than or equal to the maximum value, it will be fully lit
* @param[in2]    R    value(0~255)
* @param[in3]    G    value(0~255)
* @param[in4]    B    value(0~255)
* @param[out]    void
* @retval        void
* @par History   no
*/
void showRGB(int num, int R, int G, int B)
{
  uint32_t color = strip.Color(G, R, B);
  if (num > MAX_LED - 1)  
  {
    for (int i = 0; i < MAX_LED; i++)
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
* Function       waterfall_RGB
* @author        chengengyue
* @date          2019.10.08
* @brief         water light（green）
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
* @brief         marquee
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
* @brief         breathing light
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
* @brief         color light
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
  pinMode(RGB_PIN, OUTPUT);        //Set the RGB pin to output mode
  pinMode(KEY_PIN, INPUT_PULLUP);  //Set the button pin to pull-up input mode

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
  // put your main code here, to run repeatedly:
  // keyscan
  keyscan();
  
  switch (RGB_Effect)
  {
  case 1:     //breathing light
    nowTime = millis();
    if (nowTime - oldTime >= 20)
    {
      oldTime = nowTime;
      breathing_RGB();
    }
    break;
  
  case 2:   //marquee 
    nowTime = millis();
    if (nowTime - oldTime >= 150)
    {
      oldTime = nowTime;
      marquee_RGB();
    }
    break;
  
  case 3:   //water light  
    nowTime = millis();
    if (nowTime - oldTime >= 300)
    {
      oldTime = nowTime;
      waterfall_RGB();
    }
    break;
  
  case 4:   //colorful light  
    nowTime = millis();
    if (nowTime - oldTime >= 10)
    {
      oldTime = nowTime;
      colorfull_RGB();
    }
    break;

  case 5:   //Turn off light 
    showRGB(MAX_LED, 0, 0, 0);
    RGB_Effect = 0;
    break;

  default:
    break;
  }

}
