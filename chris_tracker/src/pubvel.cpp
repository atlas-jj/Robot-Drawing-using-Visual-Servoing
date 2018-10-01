//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>//for geometry_mesgs::Twist
//#include <stdlib.h> //for rand() and RAND_MAX
//
//int main (int argc, char ** argv){
////initialize the ROS system and become a node.
//ros::init(argc, argv, "publish_velocity");
//ros::NodeHandle nh;
//
////create a publisher object.
//ros::Publisher pub = nh.advertise <geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
//
////send the random number generator.
//srand(time(0));
//
////loop at 2Hz unit the node is shutdown.
//ros::Rate rate(2);
//while(ros::ok()){
////create and fill in the message.
//    geometry_msgs::Twist msg;
//    msg.linear.x=double(rand())/double(RAND_MAX);
//    msg.angular.z=2*double(rand())/double(RAND_MAX)-1;
//
//    //publish the message.
//    pub.publish(msg);
//
//    //send a message to rosout with the details.
//    ROS_INFO_STREAM("Sending random velocity command:"
//           <<"linear="<<msg.linear.x
//          <<"angular="<<msg.angular.z);
//
//   //wait until time for another iteration.
//   rate.sleep();
//}
//}
