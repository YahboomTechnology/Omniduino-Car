/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Serial.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Serial Input and Output
* @details
* @par History  no
*/
void setup() {
  //put your setup code here, to run once:
  //Initialize the serial port, the baud rate is 9600
  Serial.begin(9600);
  Serial.println("Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available())
  {
    Serial.print((char)Serial.read());
  }
}
