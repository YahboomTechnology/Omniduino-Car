/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Whistle.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Whistle
* @details
* @par History  no
*/
#define BUZZER 10    //Define buzzer pin
#define KEY_PIN 8    //Define key pin
#define LED_PIN 5    //Define LED(D9) pin


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

void setup() {
  // put your setup code here, to run once:
  pinMode(KEY_PIN, INPUT_PULLUP);//Set the button pin to pull-up input mode
  pinMode(LED_PIN, OUTPUT);      //Set the LED pin to output mode
  pinMode(BUZZER, OUTPUT);       //Set the buzzer pin to output mode
}

void loop() {
  // put your main code here, to run repeatedly:
  // key scan
  keyscan();
  if (button_press)
  {
    whistle();             
    button_press = false;  //set key status to false
  }
}
