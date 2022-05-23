#include "ros/ros.h" 
#include "std_msgs/Float32.h" 

// NODE NAME, LOOP RATE & TOPICS' NAMES 
#define     NODE_NAME       "arduino_volt2percent" 
#define     LOOP_RATE       2                       // 2Hz                  
#define     TOPIC1_SUB      "arduino_AI_volt"       // Sub = Subscribe [0.0 - 5.0 V]
#define     TOPIC1_PUB      "arduino_AI_percent"    // Pub = Publisher [0 - 100 %]

// Function prototypes - Subscribed topics
std_msgs::Float32 sensorValue_volt; 
void GetSensorValue_volt(const std_msgs::Float32& msg);  

int main(int argc, char **argv)
{
    // init ROS  		
    ros::init(argc, argv, NODE_NAME);   
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE); 

    // Topics
    ros::Subscriber topic1_sub = n.subscribe(TOPIC1_SUB, 50, GetSensorValue_volt);  
    ros::Publisher topic1_pub = n.advertise<std_msgs::Float32&>(TOPIC1_PUB,50); 	
    
    // Loop ROS
    std_msgs::Float32 sensorValue_percent;
    while(ros::ok()) 
    {
        // Conversion
        sensorValue_percent.data = sensorValue_volt.data * (100.0 / 5.0);
		ROS_INFO("Sensor value got from Arduino was converted from %f [Volt] to %f [%%]", sensorValue_volt.data, sensorValue_percent.data);

	    // Publishing
        topic1_pub.publish(sensorValue_percent);

	    // Loop rate
        ros::spinOnce(); 
        loop_rate.sleep(); 
    }
    return 0;
}

/* Func: GetSensorValue_volt
    Summary: Get the AI voltage from Arduino */
void GetSensorValue_volt(const std_msgs::Float32& msg) 
{
    sensorValue_volt.data = msg.data;	
}
