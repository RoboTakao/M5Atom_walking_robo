#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

#include "M5Atom.h"

const uint8_t Srv0 = 22; //GPIO Right Leg
const uint8_t Srv1 = 19; //GPIO Right Foot
const uint8_t Srv2 = 23; //GPIO Left Foot
const uint8_t Srv3 = 33; //GPIO Left Leg

const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3; //チャンネル
const double PWM_Hz = 50;   //PWM周波数
const uint8_t PWM_level = 16; //PWM 16bit(0～65535)

int pulseMIN = 1640;  //0deg 500μsec 50Hz 16bit : PWM周波数(Hz) x 2^16(bit) x PWM時間(μs) / 10^6
int pulseMAX = 8190;  //180deg 2500μsec 50Hz 16bit : PWM周波数(Hz) x 2^16(bit) x PWM時間(μs) / 10^6

int cont_min = 0;
int cont_max = 180;

int angZero[] = {90, 90, 90, 90};
int ang0[4];
int ang1[4];
int ang_b[4];
char ang_c[4];
float ts=150;  //150msごとに次のステップに移る
float td=10;   //10回で分割

// Forward Step
int f_s[19][4]={
  {0,0,0,0},
  {0,40,25,0},
  {0,25,25,0},
  {15,25,25,15},
  {15,0,0,15},
  {15,-25,-40,15},
  {15,-25,-25,15},
  {0,-25,-25,0},
  {-15,-25,-25,-15},
  {-15,0,0,-15},
  {-15,40,25,-15},
  {-15,25,25,-15},
  {0,25,25,0},
  {15,25,25,15},
  {15,0,0,15},
  {15,-25,-40,15},
  {15,-25,-25,15},
  {0,-25,-25,0},
  {0,0,0,0}};

// Back Step
int b_s[19][6]={
  {0,0,0,0},
  {0,40,25,0},
  {0,25,25,0},
  {-15,25,25,-15},
  {-15,0,0,-15},
  {-15,-25,-40,-15},
  {-15,-25,-25,-15},
  {0,-25,-25,0},
  {15,-25,-25,15},
  {15,0,0,15},
  {15,40,25,15},
  {15,25,25,15},
  {0,25,25,0},
  {-15,25,25,-15},
  {-15,0,0,-15},
  {-15,-25,-40,-15},
  {-15,-25,-25,-15},
  {0,-25,-25,0},
  {0,0,0,0}};

// Left Turn_Step
int l_s[9][4]={
  {0,0,0,0},
  {0,-25,-40,0},
  {0,-25,-25,0},
  {15,-25,-25,-15},
  {15,0,0,-15},
  {15,40,25,-15},
  {15,25,25,-15},
  {0,25,25,0},
  {0,0,0,0}};

// Right Turn Step
int r_s[9][4]={
  {0,0,0,0},
  {0,40,25,0},
  {0,25,25,0},
  {15,25,25,-15},
  {15,0,0,-15},
  {15,-25,-40,-15},
  {15,-25,-25,-15},
  {0,-25,-25,0},
  {0,0,0,0}};

void Initial_Value(){  //initial servo angle
  for (int j=0; j <=3 ; j++){
      ang0[j] = angZero[j];
  }
  for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j];
  }
  servo_set();
}

void Srv_drive(int srv_CH,int SrvAng){
  SrvAng = map(SrvAng, cont_min, cont_max, pulseMIN, pulseMAX);
  ledcWrite(srv_CH, SrvAng);
}

void forward_step()
{
  for (int i=0; i <=18 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + f_s[i][j];
    }
  servo_set();
  }
}

void back_step()
{
  for (int i=0; i <=18 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + b_s[i][j];
    }
  servo_set();
  }
}

void right_step()
{
  for (int i=0; i <=8 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + r_s[i][j];
    }
  servo_set();
  }
}

void left_step()
{
  for (int i=0; i <=8 ; i++){
    for (int j=0; j <=3 ; j++){
      ang1[j] = angZero[j] + l_s[i][j];
    }
  servo_set();
  }
}

void servo_set(){     //線形補完してサーボに指令値を送る関数
  int a[4],b[4];
  
  for (int j=0; j <=3 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }

  for (int k=0; k <=td ; k++){

      Srv_drive(srv_CH0, a[0]*float(k)/td+b[0]);
      Srv_drive(srv_CH1, a[1]*float(k)/td+b[1]);
      Srv_drive(srv_CH2, a[2]*float(k)/td+b[2]);
      Srv_drive(srv_CH3, a[3]*float(k)/td+b[3]);

      delay(ts/td);
  }
}

void setup() {
  M5.begin(true, false, true); //SerialEnable , I2CEnable , DisplayEnable 
  Dabble.begin("M5Atom");       //set bluetooth name of your device
  
  pinMode(Srv0, OUTPUT);
  pinMode(Srv1, OUTPUT);
  pinMode(Srv2, OUTPUT);
  pinMode(Srv3, OUTPUT);
  
  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(srv_CH0, PWM_Hz, PWM_level);
  ledcSetup(srv_CH1, PWM_Hz, PWM_level);
  ledcSetup(srv_CH2, PWM_Hz, PWM_level);
  ledcSetup(srv_CH3, PWM_Hz, PWM_level);

  //モータのピンとチャンネルの設定
  ledcAttachPin(Srv0, srv_CH0);
  ledcAttachPin(Srv1, srv_CH1);
  ledcAttachPin(Srv2, srv_CH2);
  ledcAttachPin(Srv3, srv_CH3);

  Initial_Value();

  M5.dis.drawpix(0, 0x000000);  //Dummy
  M5.dis.drawpix(0, 0x0000ff);  //blue 0x0000ff
}

void loop() {
  if ( M5.Btn.wasReleased() ) {
    Initial_Value();
  }

  Dabble.processInput();             //this function is used to refresh data obtained from smartphone.Hence calling this function is mandatory in order to get data properly from your mobile.

    if (GamePad.isUpPressed())
  {
    forward_step();
    Serial.println("FWD");
  }

  if (GamePad.isDownPressed())
  {
    back_step();
    Serial.println("BACK");
  }

  if (GamePad.isLeftPressed())
  {
    left_step();
    Serial.println("LEFT STEP");
  }

  if (GamePad.isRightPressed())
  {
    right_step();
    Serial.println("RIGHT STEP");
  }

  delay(100);
}
