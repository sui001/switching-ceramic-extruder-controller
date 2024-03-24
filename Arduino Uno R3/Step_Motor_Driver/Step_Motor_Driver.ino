 // modified by Xinbo Qu, Project AAA. Feb 2024. All rights Reserved.
// modify log: get average signal intensity value for both real nRF24L01 data and the area persentage on the SSD1306 OLED screen.
// modify PORTB reg operation into specific pin operation to adapt new Uno R4. 
// modify _NOP() into __asm__ __volatile__("nop") to achieve same effect of delay for a very short time period
// add threshold and stepper switch control by digital output

// update: https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds
// delaymicroseconds are suitable for 3-16383 microseconds

// update: add dip switch control (pulse override), update debouncer design, current priority is pulse1 > pulse2 > pulse3, one output a time. 
// NOTE: adjust the minimum_pulse_delay and maximum_pulse_delay to fit your motor driver pulse speed. the range should from 3 to 16383


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
 * 1 GND      ---- Right   
 * 2 5V       ---- Left
 * 3 A5(D19)  ---- Middle
 * 
 * Uno R3   Reverse Button
 * 1 GND ---- One side
 * 2 D2  ---- The other side
 * 
 * Uno R3   Master Button
 * 1 GND ---- One side
 * 2 D3  ---- The other side
 * 
 * Uno R3   step motor driver pulse output
 * 1 D8  ---- Output 1(Green)
 * 2 D9  ---- Output 2(Yellow)
 * 2 D10 ---- Output 3(Red)
 * 
 * Uno R3   step motor driver direction output
 * 1 D12  ---- Output 1
 * 
 * Uno R3   master switch light
 * 1 D13  ---- Output 1(White LED Positive side) light up means enable
 * 
 * Uno R3   reverse switch light
 * 1 D4  ---- Output 1(Blue LED Positive side) light up means reverse
 * 
 * Uno R3   DIP override switch
 * 1 A4(D18)  ---- DIP_1
 * 2 A3(D17)  ---- DIP_2
 * 3 A2(D16)  ---- DIP_3
 * 
 */


// control the delay of pulse cycle, small delay->fast motor speed
#define minimum_pulse_delay 50
#define maximum_pulse_delay 5000

// buttons
#define reverseSwitch 2  // Push button for reverse
#define enSwitch 3  // push button for master switch

// lights
#define en_PUL_LIGHT 13
#define dir_LIGHT 4

// output to motor driver
#define driverPUL1 8    
#define driverPUL2 9    
#define driverPUL3 10    
#define driverDIR 12

// input knob
#define spd A5     // Potentiometer

//input from uno r4
#define en_PUL1 5
#define en_PUL2 6
#define en_PUL3 7

//input from DIP switch
#define DIP_1 18  //override pulse 1
#define DIP_2 17  //override pulse 2
#define DIP_3 16  //override pulse 3

// Variables
unsigned int pd = 5000;       // Pulse Delay period
boolean setdir = LOW;         // Set default Direction
boolean setEN = LOW;          // Set default Enable status
 
// Interrupt Handler
void revmotor (){
  setdir = !setdir;
}

void enswitch (){
  setEN = !setEN;
}
 

void setup() {
  // 
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

  // set interrupt for buttons
  attachInterrupt(digitalPinToInterrupt(reverseSwitch), revmotor, FALLING);
  attachInterrupt(digitalPinToInterrupt(enSwitch), enswitch, FALLING);

  // set DIP switch as internal pullup, when switch closed(switch on) will directly pull the DIP to the ground, otherwise (switch off) maintain a stable high value
  pinMode(DIP_1, INPUT_PULLUP);
  pinMode(DIP_2, INPUT_PULLUP);
  pinMode(DIP_3, INPUT_PULLUP);
  
}
 
void loop() {
    // map the delay with Potentiometer input
    pd = map((analogRead(spd)), 0, 1023, maximum_pulse_delay, minimum_pulse_delay);
    // output direction, set direction light/master light
    digitalWrite(driverDIR,setdir);
    digitalWrite(dir_LIGHT,setdir);
    digitalWrite(en_PUL_LIGHT,setEN);

    // override(ignore master switch) or normal way to control
    if ((digitalRead(DIP_1) == LOW) || (setEN && digitalRead(en_PUL1) == HIGH)) {
      digitalWrite(driverPUL1,HIGH);
      delayMicroseconds(pd);
      digitalWrite(driverPUL1,LOW);
      delayMicroseconds(pd);
    } else if ((digitalRead(DIP_2) == LOW) || (setEN && digitalRead(en_PUL2) == HIGH)) {
      digitalWrite(driverPUL2,HIGH);
      delayMicroseconds(pd);
      digitalWrite(driverPUL2,LOW);
      delayMicroseconds(pd);
    } else if ((digitalRead(DIP_3) == LOW) || (setEN && digitalRead(en_PUL3) == HIGH)) {
      digitalWrite(driverPUL3,HIGH);
      delayMicroseconds(pd);
      digitalWrite(driverPUL3,LOW);
      delayMicroseconds(pd);
    }
}
