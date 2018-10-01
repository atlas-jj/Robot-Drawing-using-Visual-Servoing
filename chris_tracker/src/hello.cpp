#include <ros/ros.h>

int main (int argc, char **argv){
//initialize the ROS system
    ros::init (argc,argv, "hello_ros");

    //establish this program as a ROS node.
    ros::NodeHandle nh;
    //set rate
    ros::Rate rate(1);
    while(ros::ok())
    {
    //send some output as a log message.
    ROS_INFO_STREAM("Hello, ROS!");
    rate.sleep();
    }
}
