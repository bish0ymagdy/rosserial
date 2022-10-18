#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
std_msgs::Int32 count_msg;
ros::Publisher count("count", &count_msg);

int interrupt_a = 2;
int interrupt_b = 3;
int c=0;

void ISR_a() {
  int a = digitalRead(interrupt_a);
  int b = digitalRead(interrupt_b);
  if (a != b){
    c++;
  }
  else{
    c--;
  }
}

void ISR_b() {
  int a = digitalRead(interrupt_a);
  int b = digitalRead(interrupt_b);
  if (a == b){
    c++;
  }
  else{
    c--;
  }
}

void setup() {
  nh.initNode();
  nh.advertise(count);
  pinMode(interrupt_a,INPUT_PULLUP);
  pinMode(interrupt_b,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interrupt_a),ISR_a,CHANGE);
  attachInterrupt(digitalPinToInterrupt(interrupt_b),ISR_b,CHANGE);

}

void loop() {
  count_msg.data = c;
  count.publish( &count_msg );
  nh.spinOnce();
  delay(1000);

}
