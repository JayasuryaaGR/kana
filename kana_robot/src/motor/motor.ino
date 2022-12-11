#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Arduino.h>
#include <CytronMotorDriver.h>

ros::NodeHandle nh;
std_msgs::Float32 max_vel_L, max_vel_R;

CytronMD L_motor(PWM_DIR, 9, 8);     // PWM 1 = Pin 9, DIR 1 = Pin 8.
CytronMD R_motor(PWM_DIR, 11, 10);   // PWM 2 = Pin 11, DIR 2 = Pin 10.

float req_pwm_L, req_pwm_R;
float percent_L, percent_R;

int encLA = 2;
int encLB = 3;
int encRA = 4;
int encRB = 5;

std_msgs::Int32 encLMsg;
std_msgs::Int32 encRMsg;

ros::Publisher EncLVal("EncLVal", &encLMsg);
ros::Publisher EncRVal("EncRVal", &encRMsg);

int encLVal = 0;
int encRVal = 0;

int encLALast = LOW;
int encRALast = LOW;
int encLAPres = LOW;
int encRAPres = LOW;

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

void setup()
{
  nh.initNode();

  max_vel_L.data = 0.5;
  max_vel_R.data = 0.5;

  //  nh.getParam("~max_vel_Fl", &max_vel_L.data);
  //  nh.getParam("~max_vel_Fr", &max_vel_R.data);

  nh.advertise(EncLVal);
  nh.advertise(EncRVal);

  pinMode(encLA, INPUT);
  pinMode(encLB, INPUT);
  pinMode(encRA, INPUT);
  pinMode(encRB, INPUT);

  nh.subscribe(lwheel_vtarget);
  nh.subscribe(rwheel_vtarget);
}

void loop()
{
  encLAPres = digitalRead(encLA);
  encRAPres = digitalRead(encRA);

  if (encLALast == LOW && encLAPres == HIGH) {
    if (digitalRead(encLB) == HIGH)
      encLVal++;
    else
      encLVal--;

    if (encLVal > 32767)
      encLVal = -32767;
    if (encLVal < -32767)
      encLVal = 32767;
  }

  if (encRALast == LOW && encRAPres == HIGH) {
    if (digitalRead(encRB) == HIGH)
      encRVal++;
    else
      encRVal--;

    if (encRVal > 32767)
      encRVal = -32767;
    if (encRVal < -32767)
      encRVal = 32767;
  }

  encLMsg.data = encLVal;
  encRMsg.data = encRVal;

  EncLVal.publish(&encLMsg);
  EncRVal.publish(&encRMsg);

  encLALast = encLAPres;
  encRALast = encRAPres;

  nh.spinOnce();
}
