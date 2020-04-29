#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;


int main(int argc, char** argv) {

    ros::init( argc, argv, "motor_vel_pub" );
    ros::NodeHandle n;

    ros::Publisher motor_vel_pub = n.advertise<std_msgs::Float32MultiArray>("/tarot_standalone/motor_vel", 0);

    std_msgs::Float32MultiArray m_data;
    m_data.data.resize(4);

    ros::Rate r(10);
    while(ros::ok()) {
        m_data.data[0] = 300;
        m_data.data[1] = 300;
        m_data.data[2] = 300;
        m_data.data[3] = 300;
        motor_vel_pub.publish( m_data );
        r.sleep();
    }



    return 0;
}