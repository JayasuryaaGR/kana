#define ROSSERIAL_ARDUINO_TCP
#define CAMERA_MODEL_AI_THINKER
#include "esp_camera.h"
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Arduino.h>
#include <CytronMotorDriver.h>

IPAddress server(192, 168, 55, 199);
const uint16_t serverPort = 11411;
const char*  ssid = "JS";
const char*  password = "GrJaya_wifi";

ros::NodeHandle nh;
std_msgs::Float32 max_vel_L, max_vel_R;

CytronMD L_motor(PWM_DIR, 2, 4);   // PWM 1 = Pin 2, DIR 1 = Pin 4.
CytronMD R_motor(PWM_DIR, 1, 3);   // PWM 2 = Pin 16, DIR 2 = Pin 17.

float req_pwm_L, req_pwm_R;
float percent_L, percent_R;

const int encLA = 12;
const int encLB = 13;
const int encRA = 14;
const int encRB = 15;

std_msgs::Int32 encLMsg;
std_msgs::Int32 encRMsg;

ros::Publisher EncLVal("EncLVal", &encLMsg);
ros::Publisher EncRVal("EncRVal", &encRMsg);

volatile int encLVal = 0;
volatile int encRVal = 0;

void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED)
    delay(500);
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());
}

void lwheel_callback(const std_msgs::Float32& inp) {
  percent_L = inp.data / max_vel_L.data;
  req_pwm_L = (int)(255 * percent_L);
  L_motor.setSpeed(req_pwm_L);
}

void rwheel_callback(const std_msgs::Float32& inp) {
  percent_R = inp.data / max_vel_R.data;
  req_pwm_R = (int)(255 * percent_R);
  R_motor.setSpeed(req_pwm_R);
}

ros::Subscriber<std_msgs::Float32> lwheel_vtarget("/lwheel_vtarget", lwheel_callback);
ros::Subscriber<std_msgs::Float32> rwheel_vtarget("/rwheel_vtarget", rwheel_callback);

void IRAM_ATTR leftEncoder()
{
  if (digitalRead(encLB) == HIGH)
    encLVal++;
  else
    encLVal--;
}

void IRAM_ATTR rightEncoder()
{
  if (digitalRead(encRB) == HIGH)
    encRVal++;
  else
    encRVal--;
}

void setup()
{
  Serial.begin(115200);
  setupWiFi();
  
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  max_vel_L.data = 1.4;
  max_vel_R.data = 1.4;

  nh.advertise(EncLVal);
  nh.advertise(EncRVal);

  pinMode(encLA, INPUT);
  pinMode(encLB, INPUT);
  pinMode(encRA, INPUT);
  pinMode(encRB, INPUT);

  nh.subscribe(lwheel_vtarget);
  nh.subscribe(rwheel_vtarget);

  attachInterrupt(digitalPinToInterrupt(encLA), leftEncoder, RISING);
  attachInterrupt(encRA, rightEncoder, RISING);
}

void loop()
{
  encLMsg.data = encLVal;
  encRMsg.data = encRVal;

  EncLVal.publish(&encLMsg);
  EncRVal.publish(&encRMsg);

  nh.spinOnce();
}
