#include "M5Atom.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <WiFi.h>

const uint8_t Srv0 = 33, Srv1 = 23, Srv2 = 19;//GPIO No.
const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2; //チャンネル
const double PWM_Hz = 50;   //PWM周波数
const uint8_t PWM_level = 16; //PWM 16bit(0～65535)

int pulseMIN = 1640;  //0deg 500μsec 50Hz 16bit
int pulseMAX = 8190;  //180deg 2500μsec 50Hz 16bit

float cont_min = -1570;
float cont_max = 1570;
int SrvAng[3] = {4914, 4914, 4914}; //90deg
float TARGET_JOINT_POSITIONS[3] = {0, 0, 0};

bool IMU6886Flag = false;

const char* ssid = "XXXXXXXXXXXXXX";
const char* password = "XXXXXXXXXXXXX";

WiFiClient client;
IPAddress server(192, 168, 10, 14); //ROS core IP adress

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11411);   
    }
    int read() {
      return client.read();      
    }
    void write(uint8_t* data, int length) {
      for (int i = 0; i < length; i++)
        client.write(data[i]);
    }
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

ros::NodeHandle_<WiFiHardware> nh;

void servo_cb(const sensor_msgs::JointState& msg){
  int target_angle[3];
  TARGET_JOINT_POSITIONS[0] = msg.position[0];
  TARGET_JOINT_POSITIONS[1] = msg.position[1];
  TARGET_JOINT_POSITIONS[2] = msg.position[2];
  target_angle[0] = 1000 * TARGET_JOINT_POSITIONS[0];
  target_angle[1] = 1000 * TARGET_JOINT_POSITIONS[1];
  target_angle[2] = 1000 * TARGET_JOINT_POSITIONS[2];
  SrvAng[0] = map(target_angle[0], cont_min, cont_max, pulseMIN, pulseMAX);
  SrvAng[1] = map(target_angle[1], cont_min, cont_max, pulseMIN, pulseMAX);
  SrvAng[2] = map(target_angle[2], cont_min, cont_max, pulseMIN, pulseMAX);
  Serial.print(target_angle[0]);
  Serial.print(" ");
  Serial.print(SrvAng[0]);
  Serial.print(" ");
  Serial.print(target_angle[1]);
  Serial.print(" ");
  Serial.print(SrvAng[1]);
  Serial.print(" ");
  Serial.print(target_angle[2]);
  Serial.print(" ");
  Serial.println(SrvAng[2]);
  ledcWrite(srv_CH0, SrvAng[0]);
  ledcWrite(srv_CH1, SrvAng[1]);
  ledcWrite(srv_CH2, SrvAng[2]);
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    Serial.print("Could not connect to"); Serial.println(ssid);
    while (1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void setup()
{ 
  M5.begin(true, false, true);
  setupWiFi();

  pinMode(Srv0, OUTPUT);
  pinMode(Srv1, OUTPUT);
  pinMode(Srv2, OUTPUT);
  
  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(srv_CH0, PWM_Hz, PWM_level);
  ledcSetup(srv_CH1, PWM_Hz, PWM_level);
  ledcSetup(srv_CH2, PWM_Hz, PWM_level);

  //モータのピンとチャンネルの設定
  ledcAttachPin(Srv0, srv_CH0);
  ledcAttachPin(Srv1, srv_CH1);
  ledcAttachPin(Srv2, srv_CH2);

  ledcWrite(srv_CH0, SrvAng[0]);
  ledcWrite(srv_CH1, SrvAng[1]);
  ledcWrite(srv_CH2, SrvAng[2]);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
