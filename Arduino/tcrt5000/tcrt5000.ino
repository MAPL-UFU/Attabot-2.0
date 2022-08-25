//sempre que houver um evento de quase colisao envia um sinal 

#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>

const int pin_SENSOR = 12;

const int pin_LED =  13;

int sensorState = 0;

int prestate = 0;

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char event[13] = "EVENT DETECT";

void setup() {

  pinMode(pin_LED, OUTPUT);

  pinMode(pin_SENSOR, INPUT);

  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(chatter);

}

void loop() {

  sensorState = digitalRead(pin_SENSOR);

  if (sensorState == HIGH && prestate == 0) {

    digitalWrite(pin_LED, HIGH);

    prestate = 1;

    str_msg.data = event;
    chatter.publish( &str_msg );
    nh.spinOnce();

    delay(500);

  } else if (sensorState == LOW) {

    digitalWrite(pin_LED, LOW);

    prestate = 0;

  }

}
