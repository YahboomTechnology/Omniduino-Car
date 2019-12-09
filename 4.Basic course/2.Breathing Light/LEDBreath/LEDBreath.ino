/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         LEDBreath.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        LED Breathing Light
* @details
* @par History  no
*/
//Define LED light(D9)pin
#define LED_PIN 5   

/* Define global variables */
int brightness = 0; //LED brightness value
int fadeAmount = 5; //Increase or decrease the brightness value each time

void setup() {
  //put your setup code here, to run once:
  //set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  //put your main code here, to run repeatedly:
  //Write analog values to the LED_PIN pin
  analogWrite(LED_PIN, brightness);

  //Modify the brightness of the next display
  brightness = brightness + fadeAmount;

  //Flip the fadeAmount value
  //If brightness >= 255, set fadeAmount to a negative value, so reduce the brightness each time
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }

  delay(30);    
}
