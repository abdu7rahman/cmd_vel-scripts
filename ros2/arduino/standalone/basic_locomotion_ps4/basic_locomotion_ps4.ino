/*
   rosserial Subscriber Example
   Blinks an LED on callback
*/

#include <ros.h>
#include <std_msgs/Int64MultiArray.h>
int l = 0, r = 0, f = 0, b = 0, p1 = 8, p2 = 9, p3 = 10, p4 = 11;
ros::NodeHandle  nh;

void messageCb( const std_msgs::Int64MultiArray& cmd_msg) {

  l = cmd_msg.data[0];
  r = cmd_msg.data[1];
  f = cmd_msg.data[2];
  b = cmd_msg.data[3];

  if (l >= 40) {
    M1(1,0,l);
    M2(1,0,l);
    M3(1,0,l);
    M4(1,0,l);
    nh.loginfo("left");
  }
  else if (r >= 40) {
    M1(1,0,r);
    M2(1,0,r);
    M3(1,0,r);
    M4(1,0,r);
    nh.loginfo("right");
  }
  else if (f >= 40) {
    M1(1,0,f);
    M2(1,0,f);
    M3(1,0,f);
    M4(1,0,f);
    nh.loginfo("front");
  }
  else if (b >= 40) {
    M1(1,0,b);
    M2(1,0,b);
    M3(1,0,b);
    M4(1,0,b);
    nh.loginfo("back");
  }
  else {
    M1(1,1,0);
    M2(1,1,0);
    M3(1,1,0);
    M4(1,1,0);
    nh.loginfo("stop");
  }
}

ros::Subscriber<std_msgs::Int64MultiArray> sub("cmd_vel", &messageCb );

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);

  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(11, OUTPUT);

}

void loop()
{
  nh.spinOnce();
  delay(1);
}

void M1(bool x, bool y, int z)
{
  digitalWrite(30, x);
  digitalWrite(32, y);
  analogWrite(p1, z);
}
void M2(bool x, bool y, int z)
{
  digitalWrite(34, x);
  digitalWrite(36, y);
  analogWrite(p2, z);
}
void M3(bool x,bool y, int z)
{
  digitalWrite(38, x);
  digitalWrite(40, y);
  analogWrite(p3, z);
}
void M4(bool x,bool y, int z)
{
  digitalWrite(42, x);
  digitalWrite(44, y);
  analogWrite(p4, z);
}
