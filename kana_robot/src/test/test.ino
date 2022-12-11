#define ROSSERIAL_ARDUINO_TCP
#include <WiFi.h>
#include <ros.h>
#include <std_msgs/Int32.h>

IPAddress server(192, 168, 55, 199);
const uint16_t serverPort = 11411;
const char*  ssid = "JS";
const char*  password = "GrJaya_wifi";

int num = 0;
std_msgs::Int32 Num;

ros::NodeHandle  nh;
ros::Publisher Hello("Hello", &Num);

void callback(const std_msgs::Int32& num) {
  Serial.println(num.data);
}

ros::Subscriber<std_msgs::Int32> Rec("rec", callback);

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

void setup(){
    Serial.begin(115200);
    setupWiFi();
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    nh.advertise(Hello);
    nh.subscribe(Rec);
}

void loop(){
  delay(100);
  num++;
  Num.data = num;
  Serial.println(Num.data);
  Hello.publish(&Num);
  nh.spinOnce();
}
