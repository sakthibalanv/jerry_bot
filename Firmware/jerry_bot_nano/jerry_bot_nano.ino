#include "PinChangeInterrupt.h"

#include <ros.h>
#include <jerry_bot_driver/uc_states.h>

#define R_EN_A 2
#define R_EN_B 5

#define L_EN_A 3
#define L_EN_B 4

#define R_M_Speed 6
#define R_M_A 8
#define R_M_B 7

#define L_M_Speed 11
#define L_M_A 9
#define L_M_B 10


// counts per rev found was 2520, i.e 90 ratio gearbox for 200rpm
// 28 TICKS PER REV ON MOTOR SIDE.
// rated torque 6.5kgcm
// L motor 22.16rad/sec -22.6rad/sec
// L motor 158 rad/sec2 -163rad/sec2

// R motor 21.4rad/sec -22.6rad/sec
// R motor 157rad/sec2 -161rad/sec2
const float TicksPerRev = 2520;
const int cycle_time = 100;

volatile long R_Ticks = 0, L_Ticks = 0;
boolean L_A_set, R_A_set;
boolean L_B_set, R_B_set;

jerry_bot_driver::uc_states joint_state;
double R_Motor_rad_sec_target = 0, R_Motor_POS = 0, R_Motor_Vel = 0;
double L_Motor_rad_sec_target = 0, L_Motor_POS = 0, L_Motor_Vel = 0;


long L_newTicks = 0, L_oldTicks = 0;
long R_newTicks = 0, R_oldTicks = 0;

//long vel_timer_c = 0, R_Old_Ticks = 0;

void cmd_cb( const jerry_bot_driver::uc_states& msg){
  L_Motor_rad_sec_target = msg.data[0]; 
  R_Motor_rad_sec_target = msg.data[1];
}


ros::NodeHandle  nh;
ros::Subscriber<jerry_bot_driver::uc_states> sub("/jerry_bot/uc_cmds", cmd_cb);
ros::Publisher pub("/jerry_bot/uc_states", &joint_state);

unsigned long counter = 0;

const int counter_ms = 10;

float INC_LMT = 250;
float LKp = 25, LKi = 0.2, LKd = 0.0; 
float RKp = 25, RKi = 0.2, RKd = 0.0;

float lError, lPid, lIncError, oldErrorL;
float rError, rPid, rIncError, oldErrorR;

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

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);

  counter = millis() + counter_ms;
}

void loop() 
{
  if(counter < millis())
  {
    R_newTicks = R_Ticks;
    L_newTicks = L_Ticks;
    
    counter = millis() + counter_ms;
    // v = (change in ticks / Ticks per rev) / 1
    R_Motor_Vel = ((R_newTicks - R_oldTicks)*1000*6.28/ (TicksPerRev*counter_ms)); // rps
    L_Motor_Vel = ((L_newTicks - L_oldTicks)*1000*6.28/ (TicksPerRev*counter_ms)); // rps

    R_Motor_POS = (R_Ticks / TicksPerRev) * 6.28;
    L_Motor_POS = (L_Ticks / TicksPerRev) * 6.28;
    
    R_oldTicks = R_newTicks;
    L_oldTicks = L_newTicks; 

    R_Move(R_Motor_rad_sec_target);
    L_Move(L_Motor_rad_sec_target);

    joint_state.data_length = 4;
    float dt[] = {L_Motor_POS, L_Motor_Vel, R_Motor_POS, R_Motor_Vel};
    joint_state.data = dt;
    pub.publish(&joint_state);
    nh.spinOnce();
  }

}

void R_Move(float vel)
{
  if(vel == 0)
    rIncError = 0;
    
  rError = vel - R_Motor_Vel;
  rIncError += rError;
  rIncError = rIncError > INC_LMT ? INC_LMT : rIncError < -INC_LMT ? -INC_LMT : rIncError;
  rPid = (RKp * rError) + (RKi * rIncError) + (RKd * (oldErrorR - rError));
  oldErrorR = rError; 
  
  if(rPid < 0)
  {
    digitalWrite(R_M_B, LOW);
    digitalWrite(R_M_A, HIGH);
    rPid = rPid * -1;
  }
  else
  {
    digitalWrite(R_M_A, LOW);
    digitalWrite(R_M_B, HIGH);
  }
  rPid = rPid > 255 ? 255 : rPid;
  analogWrite(R_M_Speed, rPid);
}

void L_Move(float vel)
{
  if(vel == 0)
    lIncError = 0;
    
  lError = vel - L_Motor_Vel;
  lIncError += lError;
  lIncError = lIncError > INC_LMT ? INC_LMT : lIncError < -INC_LMT ? -INC_LMT : lIncError;
  lPid = (LKp * lError) + (LKi * lIncError) + (LKd * (oldErrorL - lError));
  oldErrorL = lError; 
  
  if(lPid < 0)
  {
    digitalWrite(L_M_B, LOW);
    digitalWrite(L_M_A, HIGH);
    lPid = lPid * -1;
  }
  else
  {
    digitalWrite(L_M_A, LOW);
    digitalWrite(L_M_B, HIGH);
  }
  lPid = lPid > 255 ? 255 : lPid;
  analogWrite(L_M_Speed, lPid);
}

void R_EN_A_CB() {
  R_A_set = digitalRead(R_EN_A) == HIGH;
  R_Ticks += (R_A_set != R_B_set) ? -1 : +1;
}

void R_EN_B_CB() {
  R_B_set = digitalRead(R_EN_B) == HIGH;
  R_Ticks += (R_A_set == R_B_set) ? -1 : +1;
}

void L_EN_A_CB() {
  L_A_set = digitalRead(L_EN_A) == HIGH;
  L_Ticks += (L_A_set != L_B_set) ? +1 : -1;
}

void L_EN_B_CB() {
  L_B_set = digitalRead(L_EN_B) == HIGH;
  L_Ticks += (L_A_set == L_B_set) ? +1 : -1;
}
