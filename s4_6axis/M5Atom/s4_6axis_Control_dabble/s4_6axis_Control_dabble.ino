#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>

#include "M5Atom.h"

const uint8_t Srv0 = 22; //GPIO Right Arm
const uint8_t Srv1 = 19; //GPIO Right Leg
const uint8_t Srv2 = 23; //GPIO Right Foot
const uint8_t Srv3 = 33; //GPIO Left Foot
const uint8_t Srv4 = 25; //GPIO Left Leg
const uint8_t Srv5 = 21; //GPIO Left Arm

const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3, srv_CH4 = 4, srv_CH5 = 5; //チャンネル
const double PWM_Hz = 50;   //PWM周波数
const uint8_t PWM_level = 16; //PWM 16bit(0～65535)

int pulseMIN = 1640;  //0deg 500μsec 50Hz 16bit : PWM周波数(Hz) x 2^16(bit) x PWM時間(μs) / 10^6
int pulseMAX = 8190;  //180deg 2500μsec 50Hz 16bit : PWM周波数(Hz) x 2^16(bit) x PWM時間(μs) / 10^6

int cont_min = 0;
int cont_max = 180;

int angZero[] = {90, 78, 85, 88, 92, 90};
int ang0[6];
int ang1[6];
int ang_b[6];
char ang_c[6];
float ts=160;  //160msごとに次のステップに移る
float td=20;   //20回で分割

// Forward Step
int f_s[19][6]={
  {0,0,0,0,0,0},
  {0,0,-15,-10,0,0},
  {0,0,-15,-15,0,0},
  {-20,15,-15,-15,15,-20},
  {-20,15,0,0,15,-20},
  {-20,15,10,15,15,-20},
  {-20,15,15,15,15,-20},
  {0,0,15,15,0,0},
  {20,-15,15,15,-15,20},
  {20,-15,0,0,-15,20},
  {20,-15,-15,-10,-15,20},
  {20,-15,-15,-15,-15,20},
  {0,0,-15,-15,0,0},
  {-20,15,-15,-15,15,-20},
  {-20,15,0,0,15,-20},
  {-20,15,10,15,15,-20},
  {-20,15,15,15,15,-20},
  {0,0,15,15,0,0},
  {0,0,0,0,0,0}};

// Back Step
int b_s[19][6]={
  {0,0,0,0,0,0},
  {0,0,-20,-15,0,0},
  {0,0,-15,-15,0,0},
  {0,-15,-15,-15,-15,20},
  {0,-15,0,0,-15,20},
  {0,-15,15,20,-15,20},
  {0,-15,15,15,-15,20},
  {0,0,15,15,0,0},
  {0,15,15,15,15,-20},
  {0,15,0,0,15,-20},
  {0,15,-20,-15,15,-20},
  {0,15,-15,-15,15,-20},
  {0,0,-15,-15,0,0},
  {0,-15,-15,-15,-15,20},
  {0,-15,0,0,-15,20},
  {0,-15,15,20,-15,20},
  {0,-15,15,15,-15,20},
  {0,0,15,15,0,0},
  {0,0,0,0,0,0}};

// Left Turn_Step
int l_s[9][6]={
  {0,0,0,0,0,0},
  {0,0,10,15,0,0},
  {0,0,15,15,0,0},
  {-20,15,15,15,-15,-20},
  {-20,15,0,0,-15,-20},
  {-20,15,-15,-10,-15,-20},
  {-20,15,-15,-15,-15,-20},
  {0,0,-15,-15,0,0},
  {0,0,0,0,0,0}};

// Right Turn Step
int r_s[9][6]={
  {0,0,0,0,0,0},
  {0,0,-15,-10,0,0},
  {0,0,-15,-15,0,0},
  {-20,15,-15,-15,-15,-20},
  {-20,15,0,0,-15,-20},
  {-20,15,10,15,-15,-20},
  {-20,15,15,15,-15,-20},
  {0,0,15,15,0,0},
  {0,0,0,0,0,0}};

// Right Arm
int r_a[7][6]={
  {0,0,0,0,0,0},
  {80,0,0,0,0,0},
  {0,0,0,0,0,0},
  {80,0,0,0,0,0},
  {0,0,0,0,0,0},
  {80,0,0,0,0,0},
  {0,0,0,0,0,0}};

// Left Arm
int l_a[7][6]={
  {0,0,0,0,0,0},
  {0,0,0,0,0,-80},
  {0,0,0,0,0,0},
  {0,0,0,0,0,-80},
  {0,0,0,0,0,0},
  {0,0,0,0,0,-80},
  {0,0,0,0,0,0}};

int direction = 0;

void Initial_Value(){  //initial servo angle
  for (int j=0; j <=5 ; j++){
      ang0[j] = angZero[j];
  }
  for (int j=0; j <=5 ; j++){
      ang1[j] = angZero[j];
  }
  servo_set();
}

void face_clear(){
  for(int i=0; i<25; i++){
    M5.dis.drawpix(i, 0x000000); //black
    //M5.dis.drawpix(i, 0xa5ff00); //orange
  }
}

void face_center(){
  M5.dis.drawpix(6, 0x00ff00);  //red
  M5.dis.drawpix(7, 0x00ff00);
  M5.dis.drawpix(8, 0x00ff00);
  M5.dis.drawpix(16, 0x0000ff);  //blue 0x0000ff
  M5.dis.drawpix(18, 0x0000ff);
}

void face_right(){
  face_clear();
  M5.dis.drawpix(7, 0x00ff00);
  M5.dis.drawpix(8, 0x00ff00);
  M5.dis.drawpix(9, 0x00ff00);
  M5.dis.drawpix(17, 0x0000ff);
  M5.dis.drawpix(19, 0x0000ff);
}

void face_left(){
  face_clear();
  M5.dis.drawpix(5, 0x00ff00);
  M5.dis.drawpix(6, 0x00ff00);
  M5.dis.drawpix(7, 0x00ff00);
  M5.dis.drawpix(15, 0x0000ff);
  M5.dis.drawpix(17, 0x0000ff);
}

void Srv_drive(int srv_CH,int SrvAng){
  SrvAng = map(SrvAng, cont_min, cont_max, pulseMIN, pulseMAX);
  ledcWrite(srv_CH, SrvAng);
}

void forward_step()
{
  for (int i=0; i <=18 ; i++){
    for (int j=0; j <=5 ; j++){
      ang1[j] = angZero[j] + f_s[i][j];
    }
  servo_set();
  }
}

void back_step()
{
  for (int i=0; i <=18 ; i++){
    for (int j=0; j <=5 ; j++){
      ang1[j] = angZero[j] + b_s[i][j];
    }
  servo_set();
  }
}

void right_step()
{
  face_right();
  for (int i=0; i <=8 ; i++){
    for (int j=0; j <=5 ; j++){
      ang1[j] = angZero[j] + r_s[i][j];
    }
  servo_set();
  }
  face_clear();
  face_center();
}

void left_step()
{
  face_left();
  for (int i=0; i <=8 ; i++){
    for (int j=0; j <=5 ; j++){
      ang1[j] = angZero[j] + l_s[i][j];
    }
  servo_set();
  }
  face_clear();
  face_center();
}

void right_arm()
{
  face_right();
  for (int i=0; i <=6 ; i++){
    for (int j=0; j <=5 ; j++){
      ang1[j] = angZero[j] + r_a[i][j];
    }
  servo_set();
  }
  face_clear();
  face_center();
}

void left_arm()
{
  face_left();
  for (int i=0; i <=6 ; i++){
    for (int j=0; j <=5 ; j++){
      ang1[j] = angZero[j] + l_a[i][j];
    }
  servo_set();
  }
  face_clear();
  face_center();
}

void servo_set(){
  int a[6],b[6];
  
  for (int j=0; j <=5 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];
  }

  for (int k=0; k <=td ; k++){

      Srv_drive(srv_CH0, a[0]*float(k)/td+b[0]);
      Srv_drive(srv_CH1, a[1]*float(k)/td+b[1]);
      Srv_drive(srv_CH2, a[2]*float(k)/td+b[2]);
      Srv_drive(srv_CH3, a[3]*float(k)/td+b[3]);
      Srv_drive(srv_CH4, a[4]*float(k)/td+b[4]);
      Srv_drive(srv_CH5, a[5]*float(k)/td+b[5]);

      delay(ts/td);
  }
}

void setup() {
  Serial.begin(151200);
  // void M5Atom::begin(bool SerialEnable , bool I2CEnable , bool DisplayEnable )
  M5.begin(true, false, true);
  Dabble.begin("M5Atom2");       //set bluetooth name of your device

  pinMode(Srv0, OUTPUT);
  pinMode(Srv1, OUTPUT);
  pinMode(Srv2, OUTPUT);
  pinMode(Srv3, OUTPUT);
  pinMode(Srv4, OUTPUT);
  pinMode(Srv5, OUTPUT);
  
  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(srv_CH0, PWM_Hz, PWM_level);
  ledcSetup(srv_CH1, PWM_Hz, PWM_level);
  ledcSetup(srv_CH2, PWM_Hz, PWM_level);
  ledcSetup(srv_CH3, PWM_Hz, PWM_level);
  ledcSetup(srv_CH4, PWM_Hz, PWM_level);
  ledcSetup(srv_CH5, PWM_Hz, PWM_level);

  //モータのピンとチャンネルの設定
  ledcAttachPin(Srv0, srv_CH0);
  ledcAttachPin(Srv1, srv_CH1);
  ledcAttachPin(Srv2, srv_CH2);
  ledcAttachPin(Srv3, srv_CH3);
  ledcAttachPin(Srv4, srv_CH4);
  ledcAttachPin(Srv5, srv_CH5);

  face_center();

  Initial_Value();
}

void loop() {
  M5.update();
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
  
  if (GamePad.isSquarePressed())
  {
    left_arm();
    Serial.println("Left ARM");
  }

  if (GamePad.isCirclePressed())
  {
    right_arm();
    Serial.println("RIGHT ARM");
  }

  delay(100);
}
