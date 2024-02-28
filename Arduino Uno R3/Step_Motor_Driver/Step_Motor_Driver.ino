// modified by Xinbo Qu, Project AAA. Feb 2024. All rights Reserved.
// modify log: get average signal intensity value for both real nRF24L01 data and the area persentage on the SSD1306 OLED screen.
// modify PORTB reg operation into specific pin operation to adapt new Uno R4. 
// modify _NOP() into __asm__ __volatile__("nop") to achieve same effect of delay for a very short time period
// add threshold and stepper switch control by digital output


/*
 * Arduino Uno R3 Connection
 * 
 * Uno R3   Arduino Uno R4 Wi-Fi
 * 1 GND ---- GND   Note: connect GND can prevent from ground offset
 * 2 D5  ---- D5
 * 3 D6  ---- D6
 * 4 D7  ---- D7
 */


 /*
 * Arduino Uno R3 Connection
 * 
 * Uno R3   Potentiometer
 * 1 GND ---- Right   
 * 2 5V  ---- Left
 * 3 A5  ---- Middle
 * 
 * Uno R3   Button
 * 1 GND ---- One side
 * 2 D2  ---- The other side
 * 
 * Uno R3   step motor driver pulse output
 * 1 D8  ---- Output 1(Green)
 * 2 D9  ---- Output 2(Yellow)
 * 2 D10 ---- Output 3(Red)
 * 
 * Uno R3   step motor driver direction output
 * 1 D12  ---- Output 1(Blue)
 */


int reverseSwitch = 2;  // Push button for reverse
int driverPUL1 = 8;    // PUL- pin
int driverPUL2 = 9;    // PUL- pin
int driverPUL3 = 10;    // PUL- pin
int driverDIR = 12;    // DIR- pin
int spd = A5;     // Potentiometer
int en_PUL1 = 5;
int en_PUL2 = 6;
int en_PUL3 = 7;

int enSwitch = 3;
int en_PUL_LIGHT = 13;
int dir_LIGHT = 4;
 
// Variables
 
int pd = 500;       // Pulse Delay period
boolean setdir = LOW; // Set Direction
boolean setEN = LOW; // Set Direction
 
// Interrupt Handler
 
void revmotor (){
 
  setdir = !setdir;
  
}

void enswitch (){
 
  setEN = !setEN;
  
}
 
 
void setup() {
 
  pinMode (driverPUL1, OUTPUT);
  digitalWrite(driverPUL1,LOW);
  pinMode (en_PUL1, INPUT);
  pinMode (driverPUL2, OUTPUT);
  digitalWrite(driverPUL2,LOW);
  pinMode (en_PUL2, INPUT);
  pinMode (driverPUL3, OUTPUT);
  digitalWrite(driverPUL3,LOW);
  pinMode (en_PUL3, INPUT);
  pinMode (driverDIR, OUTPUT);
  digitalWrite(driverDIR,LOW);

  pinMode (en_PUL_LIGHT, OUTPUT);
  digitalWrite(en_PUL_LIGHT,LOW);
  pinMode (dir_LIGHT, OUTPUT);
  digitalWrite(driverDIR,LOW);
  
  attachInterrupt(digitalPinToInterrupt(reverseSwitch), revmotor, FALLING);
  attachInterrupt(digitalPinToInterrupt(enSwitch), enswitch, FALLING);
  
}
 
void loop() {
    pd = map((analogRead(spd)),0,1023,2000,50);
    digitalWrite(driverDIR,setdir);
    digitalWrite(dir_LIGHT,setdir);
    digitalWrite(en_PUL_LIGHT,setEN);
    
    if (setEN) {
      if (digitalRead(en_PUL1) == HIGH) {
        digitalWrite(driverPUL1,HIGH);
        delayMicroseconds(pd);
        digitalWrite(driverPUL1,LOW);
        delayMicroseconds(pd);
      } else if (digitalRead(en_PUL2) == HIGH) {
        digitalWrite(driverPUL2,HIGH);
        delayMicroseconds(pd);
        digitalWrite(driverPUL2,LOW);
        delayMicroseconds(pd);
      } else if (digitalRead(en_PUL3) == HIGH) {
        digitalWrite(driverPUL3,HIGH);
        delayMicroseconds(pd);
        digitalWrite(driverPUL3,LOW);
        delayMicroseconds(pd);
      }
    }
}
