#include "M5Atom.h"
#include <Wire.h>          // I2C setting
#include <PCA9685.h>       //for PCA9685
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <WiFi.h>

//Sonic Sensor
#define echo 33 // Echo Pin
#define trig 23 // Trig Pin
double Duration = 0; //recieved duration
double Distance = 0; //distance


//Input address if PCA9685
PCA9685 pwm = PCA9685(0x40);

#define SERVOMIN 104            //Min pulse width (12bit 500μs) 
#define SERVOMAX 512            //Max pulse width (12bit 2500μs) 

float cont_min = -1570;
float cont_max = 1570;
int SrvAng[12] = {307, 307, 307, 307, 307, 307, 307, 307, 307, 307, 307, 307}; //90deg
float TARGET_JOINT_POSITIONS[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float TRIM[12] = {0.03, -0.05, -0.03, 0.07, 0.12, -0.12, 0.03, -0.07, -0.16, -0.03, 0.1, 0.0};
int target_angle[12];

const char* ssid = "XXXXXXXXXXXXXX";
const char* password = "XXXXXXXXXXXXX";

WiFiClient client;
IPAddress server(192, 168, 10, 14); //ROS core IP adress

//IMU
float roll, pitch, yaw;
#define IMU_AFS M5.IMU.AFS_2G       // Ascale [g]      (±2,4,8,16)
#define IMU_GFS M5.IMU.GFS_250DPS  // Gscale [deg/s]  (±250,500,1000,200)
int roll_initial, pitch_initial, yaw_initial;
int initial_count = 100;

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

std_msgs::Int32MultiArray IMU_Distance_data;

void servo_cb(const sensor_msgs::JointState& msg) {

  TARGET_JOINT_POSITIONS[0] = msg.position[0]; //FrontLeft 3
  TARGET_JOINT_POSITIONS[1] = msg.position[1]; //FrontLeft 4
  TARGET_JOINT_POSITIONS[2] = msg.position[2]; //FrontLeft 5
  TARGET_JOINT_POSITIONS[3] = msg.position[3]; //RearLeft 9
  TARGET_JOINT_POSITIONS[4] = msg.position[4]; //RearLeft 10
  TARGET_JOINT_POSITIONS[5] = msg.position[5]; //RearLeft 11
  TARGET_JOINT_POSITIONS[6] = msg.position[6]; //FrontRight 0
  TARGET_JOINT_POSITIONS[7] = -msg.position[7]; //FrontRight 1
  TARGET_JOINT_POSITIONS[8] = -msg.position[8]; //FrontRight 2
  TARGET_JOINT_POSITIONS[9] = msg.position[9]; //RearRight 6
  TARGET_JOINT_POSITIONS[10] = -msg.position[10]; //RearRight 7
  TARGET_JOINT_POSITIONS[11] = -msg.position[11]; //RearRight 8

  servo_set();

}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);
ros::Publisher IMU_Distance_chatter("IMU_Distance_data", &IMU_Distance_data);

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

  M5.dis.drawpix(0, 0x00ff00);  //red
  M5.dis.drawpix(0, 0x00ff00);  //red
}

void servo_set()
{
  for (int i = 0; i <= 11; i++) {
    target_angle[i] = 1000 * (TARGET_JOINT_POSITIONS[i] + TRIM[i]);
    SrvAng[i] = map(target_angle[i], cont_min, cont_max, SERVOMIN, SERVOMAX); //angle（-1.57～1.57rad）-> pulse width（150～500）
    pwm.setPWM(i, 0, SrvAng[i]);
  }
}

void angle_init()
{
  //initail_count 分だけ捨てる
  for (int j = 1; j < initial_count; j++) {
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    delay(10);
  }
  
  //initail_count 分だけ初期値として平均値を求める
  for (int i = 1; i <= initial_count; i++) {
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    pitch_initial += pitch;
    roll_initial += roll;
    yaw_initial += yaw;
    delay(10);
  }
  pitch_initial /= initial_count;
  roll_initial /= initial_count;
  yaw_initial /= initial_count;
}

void setup()
{
  Serial.begin(19200);
  // void M5Atom::begin(bool SerialEnable , bool I2CEnable , bool DisplayEnable )
  M5.begin(true, false, true);

  pinMode(echo,INPUT); //Sonic Sensor echo
  pinMode(trig,OUTPUT); //Sonic Sensor triger

  setupWiFi();

  Wire.begin(19, 22); //SDA-19, SCL-22

  pwm.begin();                   //initial setting (for 0x40) (PCA9685)
  pwm.setPWMFreq(50);            //PWM 50Hz (for 0x40) (PCA9685)

  servo_set();

  M5.dis.drawpix(0, 0x0000ff);  //blue
  M5.dis.drawpix(0, 0x0000ff);  //blue

  Serial.println("Waiting for connections...");

  IMU_Distance_data.data_length = 4;
  IMU_Distance_data.data = (int32_t *)malloc(sizeof(int32_t) * 3);

  M5.IMU.Init();
  M5.IMU.SetGyroFsr(IMU_GFS);
  M5.IMU.SetAccelFsr(IMU_AFS);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(IMU_Distance_chatter);

  M5.dis.drawpix(0, 0xff0000); //green
  M5.dis.drawpix(0, 0xff0000); //green

  angle_init();

  for (uint32_t i = 0; i < 25; i++) {
    M5.dis.drawpix(i, 0xff0000); //green
  }

  xTaskCreatePinnedToCore(IMU_Distance_get, "IMU_Distance_get", 4096, NULL, 5, NULL, 1);
}

void IMU_Distance_get(void *pvParameters)
{
  while(1){
    M5.IMU.getAhrsData(&pitch, &roll, &yaw);
    pitch = pitch - pitch_initial;
    roll = roll - roll_initial;
    yaw = yaw - yaw_initial;

    digitalWrite(trig,HIGH); //超音波を出力
    delayMicroseconds(10); //10μsパルスを出す
    digitalWrite(trig,LOW);
    Duration = pulseIn(echo,HIGH); //センサからパルス間隔を取得
    if (Duration > 0) {
      Distance = Duration/5.77; // パルス間隔から距離を計算 mm
      if (Distance > 500)
      {
        Distance = 500;
      }
    }
    
    Serial.printf("pitch:%.2f,roll:%.2f,yaw:%.2f,Distance:%.2f\r\n", pitch, roll, yaw, Distance);
    IMU_Distance_data.data[0] = int(pitch);
    IMU_Distance_data.data[1] = int(roll);
    IMU_Distance_data.data[2] = int(yaw);
    IMU_Distance_data.data[3] = int(Distance);
    IMU_Distance_chatter.publish( &IMU_Distance_data );
    delay(200);
  }
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
