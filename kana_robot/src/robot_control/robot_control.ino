#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Arduino.h>
#include <CytronMotorDriver.h>
#include <Servo.h>

ros::NodeHandle nh;
std_msgs::Float32 max_vel_L, max_vel_R;

CytronMD L_motor(PWM_DIR, 11, 10); // PWM 1 = Pin 11, DIR 1 = Pin 10.
CytronMD R_motor(PWM_DIR, 9, 8);   // PWM 2 = Pin 9, DIR 2 = Pin 8.

CytronMD trash_motor(PWM_DIR, 2, 3); // PWM 1 = Pin 11, DIR 1 = Pin 10.
CytronMD mop_motor(PWM_DIR, 4, 5);   // PWM 2 = Pin 9, DIR 2 = Pin 8.

float req_pwm_L, req_pwm_R;
float percent_L, percent_R;

const int encLA = 18;
const int encLB = 19;
const int encRA = 21;
const int encRB = 20;

Servo myservo;

std_msgs::Int32 encLMsg;
std_msgs::Int32 encRMsg;

ros::Publisher EncLVal("EncLVal", &encLMsg);
ros::Publisher EncRVal("EncRVal", &encRMsg);

volatile int encLVal = 0;
volatile int encRVal = 0;

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

void control_callback(const std_msgs::Int32& inp) {
  if (inp.data==1)
  {
    for (int pos = 0; pos <= 90; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);
    }
    for (int pos = 90; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);
    }
  }
  else if (inp.data==2)
  {
    trash_motor.setSpeed(-200);
    delay(2000);
    trash_motor.setSpeed(0);
  }
  else if (inp.data==3)
  {
    mop_motor.setSpeed(150);
    delay(2000);
    mop_motor.setSpeed(0);
  }
}

ros::Subscriber<std_msgs::Float32> lwheel_vtarget("/lwheel_vtarget", lwheel_callback);
ros::Subscriber<std_msgs::Float32> rwheel_vtarget("/rwheel_vtarget", rwheel_callback);
ros::Subscriber<std_msgs::Int32> robot_control("/robot_control", control_callback);

void setup()
{
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
  nh.subscribe(robot_control);

  attachInterrupt(digitalPinToInterrupt(encLA), leftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(encRA), rightEncoder, RISING);

  myservo.attach(6);
}

void servo_fn()
{
  for (int pos = 0; pos <= 90; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);
  }
  for (int pos = 90; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);
  }
}
void leftEncoder()
{
  if (digitalRead(encLB) == HIGH)
    encLVal++;
  else
    encLVal--;
}

void rightEncoder()
{
  if (digitalRead(encRB) == HIGH)
    encRVal++;
  else
    encRVal--;
}

void loop()
{
  encLMsg.data = encLVal;
  encRMsg.data = encRVal;

  EncLVal.publish(&encLMsg);
  EncRVal.publish(&encRMsg);

  nh.spinOnce();
}
