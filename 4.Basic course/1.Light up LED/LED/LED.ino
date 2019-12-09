/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         LED.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Light up LED9
* @details
* @par History  
*/
//Define LED light(D9)pin
#define LED_PIN 5   

void setup() {
  // put your setup code here, to run once:
  // set LED pin to output mode
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_PIN, LOW);       //LED is on
  delay(500);                       
  digitalWrite(LED_PIN, HIGH);      //LED is off
  delay(500);                      
}
