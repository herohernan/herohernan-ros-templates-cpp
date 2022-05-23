#include <ros.h>                  
#include <std_msgs/Float32.h> // std_msgs from ROS      

// NODE NAME, LOOP RATE & TOPICS' NAMES
// NODE_NAME = name given to the rosserial node (serial_node.py) run on ROS 
#define     LOOP_RATE       500                     // 500ms = 2Hz                  
#define     SENSOR_PUB      "arduino_AI_volt"       // Pub = Publisher [0.0 - 5.0 V]

// I/O ports used
const int AIpin = A4; 
int AIvalue_raw;
float AIvalue_volt;

// ROS classes and variables used
ros::NodeHandle n;
std_msgs::Float32 sensorValue_volt;
ros::Publisher sensorValue_pub(SENSOR_PUB, &sensorValue_volt);

/** Start **/
void setup() 
{
  // init ROS
  n.initNode();
  n.advertise(sensorValue_pub);
}

/** Loop Arduino **/
void loop() 
{
  // Analog read and conversion
  AIvalue_raw = analogRead(AIpin);
  AIvalue_volt = (AIvalue_raw / 1024.0) * 5.0; 

  // Publishing 
  sensorValue_volt.data = AIvalue_volt;
  sensorValue_pub.publish(&sensorValue_volt);

  // Loop rate
  n.spinOnce();
  delay(LOOP_RATE);
}
