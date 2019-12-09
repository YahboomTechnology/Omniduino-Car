/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         SerialIR.ino
* @author       chengengyue
* @version      V1.0
* @date         2019.10.08
* @brief        Serial port print IR sensor value
* @details
* @par History  
*/
#define IR_SENSOR_L1 A3
#define IR_SENSOR_L2 A0
#define IR_SENSOR_R1 A2
#define IR_SENSOR_R2 A1
#define IR_SENSOR_MID A7

/*Define variables to save the data collected by the infrared sensor*/
int ir_L1; //Left front
int ir_L2; //Left rear
int ir_R1; //Right front
int ir_R2; //Right rear
int ir_Mid; //Front middle

void setup() {
  //put your setup code here, to run once:
  pinMode(IR_SENSOR_L1, INPUT);
  pinMode(IR_SENSOR_L2, INPUT);
  pinMode(IR_SENSOR_R1, INPUT);
  pinMode(IR_SENSOR_R2, INPUT);
  pinMode(IR_SENSOR_MID, INPUT);

  //Initialize the serial port, the baud rate is 9600
  Serial.begin(9600);
  Serial.println("Hello World!");
}

void loop() {
  //put your main code here, to run repeatedly:
  //Read the analog value of each sensor. 
  //The farther the distance is, the larger the value.
  ir_L1 = analogRead(IR_SENSOR_L1);
  ir_L2 = analogRead(IR_SENSOR_L2);
  ir_R1 = analogRead(IR_SENSOR_R1);
  ir_R2 = analogRead(IR_SENSOR_R2);
  ir_Mid = analogRead(IR_SENSOR_MID);

  //Print data
  Serial.print("Mid=");
  Serial.print(ir_Mid);
  Serial.print("\tL1=");
  Serial.print(ir_L1);
  Serial.print("\tL2=");
  Serial.print(ir_L2);
  Serial.print("\tR1=");
  Serial.print(ir_R1);
  Serial.print("\tR2=");
  Serial.print(ir_R2);
  Serial.println("");

  delay(10);
}
