#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;


int main(int argc, char** argv) {

    ros::init( argc, argv, "motor_vel_pub" );
    ros::NodeHandle n;

    std::string model_name(argv[1]);
    int motor_num = atoi( argv[2] );
    
    ros::Publisher motor_vel_pub = n.advertise<std_msgs::Float32MultiArray>(model_name + "/cmd/motor_vel", 0);

    std_msgs::Float32MultiArray m_data;
    m_data.data.resize(motor_num);

    ros::Rate r(10);
    int i=0;
    float tc = 0.0;
    while(ros::ok()) {

        m_data.data[i] = 100;

        tc += 1.0/10.0;
        if( tc > 2.0) {
            cout << "Actuating motor: " << i << endl;
            m_data.data[i] = 0;     
            i++;
            tc = 0.0;
            if( i > motor_num-1 ) i = 0;
        }
        motor_vel_pub.publish( m_data );
        r.sleep();
    }



    return 0;
}