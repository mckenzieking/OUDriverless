#include <SoftwareSerial.h>
SoftwareSerial s1(10, 11); // RX, TX    Setting up serial to communicate with Sabertooth
#define PIN_CH1       3 // Right X
#define PIN_CH2       4 // Left Y
#define PIN_CH3       5 // Right Y
#define PIN_CH4       6 // Left X

#include <PID_v1.h>
#include "qdec.h"

#define SABER_ADDR          130
#define M1_FORWARD          0x00
#define M1_BACKWARD         0x01

const byte interruptPinA = 2;
const byte PinB = 3;
volatile double state = 0;
volatile bool X = 0;

double setpoint, input, output;
double Kp = .3;
double Ki = 1;
double Kd = 0;
double angle = 0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
//attachInterrupt(digitalPinToInterrupt(interruptPinA), Read, RISING);
 

void setup() {
  // put your setup code here, to run once:
pinMode(interruptPinA, INPUT);
pinMode(PinB, INPUT);
 attachInterrupt(digitalPinToInterrupt(interruptPinA), ReadA, RISING);

Serial.begin(9600);
s1.begin(9600);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-127,127);
}

void loop() {
//Serial.println(state);
/*delay(500);
for(setpoint = 0; setpoint<50; setpoint++){
//setpoint = 0;  
PID_FUNCTION();
delay(10);
}

for(setpoint = 50; setpoint>0; setpoint--){
//setpoint = 0;  
PID_FUNCTION();
delay(10);
}*/
setpoint = 0;
PID_FUNCTION();
}


void ReadA() { //Interrupt function to read encoder and direction
X = digitalRead(PinB);
if(X == 0){
state+=1;
}
if(X == 1) {
state-=1;
}
}

void PID_FUNCTION() //Function to write value to PID loop
{

  angle = (state/100); //angle from top dead center
//  input = angle;
/*if(setpoint > state) {
  input = 5;
}
if(setpoint < state) {
  input = -5;
}*/
input = -angle-setpoint;
delay(10);
input = constrain(input, -127, 127);
//  Serial.println(angle);
  myPID.Compute();
  
 // Serial.println(setpoint);
  Serial.println(output);

  saberCom(SABER_ADDR, output >= 0 ? M1_FORWARD : M1_BACKWARD, (uint8_t) (output <= 0 ? output * -1 : output));
  delay(20);
  }

void saberCom(uint16_t addr, uint8_t command, uint8_t data) { //Function to communicate with sabertooth
  uint8_t chksum;
 chksum = (addr + command + data) & 0b01111111;
 s1.write(addr);
 s1.write(command);
 s1.write(data);
 s1.write(chksum);
}  
