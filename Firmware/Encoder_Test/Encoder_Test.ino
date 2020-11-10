#include "PinChangeInterrupt.h"

#define R_EN_A 2
#define R_EN_B 5

#define L_EN_A 3
#define L_EN_B 4

#define R_M_Speed 6
#define R_M_A 7
#define R_M_B 8

#define L_M_Speed 11
#define L_M_A 10
#define L_M_B 9

volatile long R_Ticks = 0, L_Ticks = 0;
long newTicks = 0, oldTicks = 0;
boolean L_A_set, R_A_set;
boolean L_B_set, R_B_set;

unsigned long counter = 0;

unsigned long stageDel = 5000;
int speedCounter = 0;
int speedStep = 10;

// counts per rev found was 2520, i.e 90 ratio gearbox for 200rpm
// 28 TICKS PER REV ON MOTOR SIDE.
// rated torque 6.5kgcm
// L motor 22.16rad/sec -22.6rad/sec
// L motor 158 rad/sec2 -163rad/sec2

// R motor 21.4rad/sec -22.6rad/sec
// R motor 157rad/sec2 -161rad/sec2
float vel = 0;
float oldVel = 0, maxAccel = 0, accel = 0;

void setup()
{
  pinMode(R_EN_A, INPUT);
  pinMode(R_EN_B, INPUT);
  pinMode(L_EN_A, INPUT);
  pinMode(L_EN_B, INPUT);

  pinMode(R_M_Speed ,OUTPUT);
  pinMode(R_M_A ,OUTPUT);
  pinMode(R_M_B ,OUTPUT);
  pinMode(L_M_Speed ,OUTPUT);
  pinMode(L_M_A ,OUTPUT);
  pinMode(L_M_B ,OUTPUT);

  attachInterrupt(0, R_EN_A_CB, CHANGE);
  attachPCINT(digitalPinToPCINT(R_EN_B), R_EN_B_CB, CHANGE);

  attachInterrupt(1, L_EN_A_CB, CHANGE);
  attachPCINT(digitalPinToPCINT(L_EN_B), L_EN_B_CB, CHANGE);

  Serial.begin (9600);

  
  /*digitalWrite(R_M_A, HIGH);
  digitalWrite(R_M_B, LOW);
  
  counter = millis() + 100;
  analogWrite(R_M_Speed, 255);*/

  
  
  //counter = millis();
}


void loop() 
{

  /*if(counter < millis())
  {
    newTicks = R_Ticks;
    counter = millis() + 100;
    // v = (change in ticks / Ticks per rev) / 1
    vel = ((newTicks - oldTicks)*10/ 2520.0); // rps
    accel = (vel - oldVel)*10; //rps2
    oldVel = vel;
    maxAccel = maxAccel > accel ? accel : maxAccel;
    oldTicks = newTicks;
    Serial.println(vel);    
  }*/

  

  /*if(millis() < counter + stageDel)
  {
    digitalWrite(L_M_A, LOW);
    digitalWrite(L_M_B, HIGH);
    speedCounter = (speedCounter + speedStep) > 255 ? 255 : (speedCounter + speedStep);
    Serial.print("Left FWd");
    Serial.print('\t');
    Serial.print(speedCounter);
    Serial.print('\t');    
    analogWrite(L_M_Speed, speedCounter);
  }
  else if(millis() < counter + 2 * stageDel)
  {
    digitalWrite(L_M_A, LOW);
    digitalWrite(L_M_B, HIGH);
    Serial.print("Left FWD");
    Serial.print('\t');
    speedCounter = (speedCounter - speedStep) < 0 ? 0 : (speedCounter - speedStep);
    analogWrite(L_M_Speed, speedCounter);
    Serial.print(speedCounter);
    Serial.print('\t');
  }
  else if(millis() < counter + 3 * stageDel)
  {
    digitalWrite(L_M_B, LOW);
    digitalWrite(L_M_A, HIGH);
    Serial.print("Left REV");
    Serial.print('\t');
    speedCounter = (speedCounter + speedStep) > 255 ? 255 : (speedCounter + speedStep);
    analogWrite(L_M_Speed, speedCounter);
    Serial.print(speedCounter);
    Serial.print('\t');
  }
  else if(millis() < counter + 4 * stageDel)
  {
    digitalWrite(L_M_B, LOW);
    digitalWrite(L_M_A, HIGH);
    Serial.print("Left REV");
    Serial.print('\t');
    speedCounter = (speedCounter - speedStep) < 0 ? 0 : (speedCounter - speedStep);
    analogWrite(L_M_Speed, speedCounter);
    Serial.print(speedCounter);
    Serial.print('\t');
  }
  else if(millis() < counter + 5 * stageDel)
  {
    digitalWrite(R_M_A, LOW);
    digitalWrite(R_M_B, HIGH);
    Serial.print("Right FWd");
    Serial.print('\t');
    speedCounter = (speedCounter + speedStep) > 255 ? 255 : (speedCounter + speedStep);
    analogWrite(R_M_Speed, speedCounter);
    Serial.print(speedCounter);
    Serial.print('\t');
  }
  else if(millis() < counter + 6 * stageDel)
  {
    digitalWrite(R_M_A, LOW);
    digitalWrite(R_M_B, HIGH);
    Serial.print("Right FWD");
    Serial.print('\t');
    speedCounter = (speedCounter - speedStep) < 0 ? 0 : (speedCounter - speedStep);
    analogWrite(R_M_Speed, speedCounter);
    Serial.print(speedCounter);
    Serial.print('\t');
  }
  else if(millis() < counter + 7 * stageDel)
  {
    digitalWrite(R_M_B, LOW);
    digitalWrite(R_M_A, HIGH);
    Serial.print("Right REV");
    Serial.print('\t');
    speedCounter = (speedCounter + speedStep) > 255 ? 255 : (speedCounter + speedStep);
    analogWrite(R_M_Speed, speedCounter);
    Serial.print(speedCounter);
    Serial.print('\t');
  }
  else if(millis() < counter + 8 * stageDel)
  {
    digitalWrite(R_M_B, LOW);
    digitalWrite(R_M_A, HIGH);
    Serial.print("Right REV");
    Serial.print('\t');
    speedCounter = (speedCounter - speedStep) < 0 ? 0 : (speedCounter - speedStep);
    analogWrite(R_M_Speed, speedCounter);
    Serial.print(speedCounter);
    Serial.print('\t');
  }

  
  
  delay(100);
  Serial.print(L_Ticks);
  Serial.print('\t');
  Serial.println(R_Ticks);
*/
}

void R_EN_A_CB() {
  R_A_set = digitalRead(R_EN_A) == HIGH;
  R_Ticks += (R_A_set != R_B_set) ? +1 : -1;
}

void R_EN_B_CB() {
  R_B_set = digitalRead(R_EN_B) == HIGH;
  R_Ticks += (R_A_set == R_B_set) ? +1 : -1;
}

void L_EN_A_CB() {
  L_A_set = digitalRead(L_EN_A) == HIGH;
  L_Ticks += (L_A_set != L_B_set) ? +1 : -1;
}

void L_EN_B_CB() {
  L_B_set = digitalRead(L_EN_B) == HIGH;
  L_Ticks += (L_A_set == L_B_set) ? +1 : -1;
}
