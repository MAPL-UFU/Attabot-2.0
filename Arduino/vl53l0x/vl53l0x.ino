#include <Wire.h>
#include <VL53L0X.h>

#define USE_USBCON
#include <ros.h>
#include <sensor_msgs/LaserScan.h>

//#define HIGH_SPEED
//#define HIGH_ACCURACY

// Cria uma instancia do sensor
VL53L0X sensor;

ros::NodeHandle  nh;

sensor_msgs::LaserScan lidar_msg;
ros::Publisher lidar_pub("/laser_scan", &lidar_msg);

float ranges[25] = {0};
int p = 0;
float angle_min;
float angle_max;
const float Pi = 3.14159;
bool take_measure = true;
//bool begin_measure = false;

void setup()
{
  // Inicializa a comunicacao serial
  //Serial.begin(9600);
  // Inicializa a comunicacao I2C
  Wire.begin();

  #if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
  #endif

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(lidar_pub);

  // Inicializa o sensor
  sensor.init();
//  sensor.startContinuous();
  // Define um timeout de 500mS para a leitura do sensor
  // Em caso de erro, este sera o tempo maximo de espera da resposta do sensor
  sensor.setTimeout(500);
}

void loop(){
//  while(begin_measure == true) {
  p = 0;
    
  lidar_msg.header.frame_id = "laser";
  lidar_msg.angle_min = -(12.5*Pi)/180;
  lidar_msg.angle_max = (12.5*Pi)/180;
  lidar_msg.angle_increment = (1*Pi)/180;
  lidar_msg.range_min = 0.05;          // minimum range value [m]
  lidar_msg.range_max = 1.2;           // maximum range value [m]

  unsigned long scan_start = millis();
    
  while(p <= 24) {
    if(take_measure == true) {
    ranges[p] = (float)sensor.readRangeSingleMillimeters()/1000.0;
    ++p;
    }
  }
  
  lidar_msg.scan_time = (millis()-scan_start)/1000.0;
//  lidar_msg.time_increment = 0.5;
  lidar_msg.ranges_length = p;
  lidar_msg.ranges = ranges;
  lidar_msg.header.stamp = nh.now();
  lidar_pub.publish(&lidar_msg);
  nh.advertise(lidar_pub);
  nh.spinOnce();
//  }
}

//void control_measure(){
//    if(begin_measure == false) {
//      begin_measure = true;
//    }
//    else  {
//      begin_measure = false;
//    }
//}
