
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <map>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include "ar_track_alvar_msgs/AlvarMarker.h"
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/robot/vpWireFrameSimulator.h>

#include "math_helper.h"
#include "string_convertor.h"
#include "transformation2D.h"
#include "wam_msgs/MatrixMN.h"
#include "wam_srvs/JointMove.h"
#include "sensor_msgs/JointState.h"
#include "pose.h"
#include "ibvs.h"
#include "colormod.h"

//using namespace cv;
using namespace std;
Color::Modifier c_red(Color::FG_RED);
Color::Modifier c_yellow(Color::FG_YELLOW);
Color::Modifier c_green(Color::FG_GREEN);
Color::Modifier c_default(Color::FG_DEFAULT);

int dofNum = 2;
wam_srvs::JointMove mv_srv;
ros::ServiceClient Joint_move_client;
cv::Mat current_trans;
cv::Mat current_rot;
vpFeaturePoint p; //current point

cv::Mat currentJacobian(6, dofNum, cv::DataType < double > ::type);
std::vector < double > current_Joint_pose;
bool ready_signal1 = false;
bool ready_signal2 = false;
bool ready_signal3 = false;
bool ready_signal4 = false;
bool lock = false;

bool signal_ready() {
  return ready_signal1 && ready_signal2 && ready_signal3 && ready_signal4;
}

//scan along columns
void wamToolJacobianCallback(const wam_msgs::MatrixMN::ConstPtr & jacobianMessage) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < dofNum; j++) {
      currentJacobian.at < double > (i, j) = jacobianMessage - > data[i + j * 6];
      //currentJacobian.at<double>(i,1)=jacobianMessage->data[i+3*6];
    }
  }
  ready_signal1 = true;
}

void wamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg) {
  //cout<<"wam msgs"<<endl;
  geometry_msgs::Pose thisPose = msg - > pose;
  cv::Mat t(3, 1, cv::DataType < double > ::type);
  cv::Mat R(3, 3, cv::DataType < double > ::type);

  t.at < double > (0, 0) = thisPose.position.x;
  t.at < double > (1, 0) = thisPose.position.y;
  t.at < double > (2, 0) = thisPose.position.z;

  tf::Quaternion q(thisPose.orientation.x, thisPose.orientation.y, thisPose.orientation.z, thisPose.orientation.w);
  tf::Matrix3x3 rMatrix(q);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R.at < double > (i, j) = rMatrix[i][j];
  current_trans = t; //end effector origin expressed in robot base
  current_rot = R; //from robot base to end effector
  ready_signal2 = true;
}

void wamJointsCallback(const sensor_msgs::JointState::ConstPtr & msg) {
  current_Joint_pose = msg - > position;
  //cout<<"joint pose obtained:"<<endl;//<<initial_Joint_pose[1]<<endl;
  //cout<<initial_Joint_pose[0]<<"  "<<initial_Joint_pose[1]<<"  "<<initial_Joint_pose[2]<<" "<<initial_Joint_pose[3]<<" "<< initial_Joint_pose[4]
  // <<" "<<initial_Joint_pose[5]<<" "<<initial_Joint_pose[6]<<endl;
  ready_signal3 = true;
}

void ar_pose_marker_callback(ar_track_alvar_msgs::AlvarMarkers req) {
  size_t ll = sizeof(req.markers[0]);
  int sizemarker = req.markers.size();
  //cout<<"size of markers"<<sizemarker<<endl;
  if (!req.markers.empty()) {
    for (int i = 0; i < sizemarker; i++) {
      ar_track_alvar_msgs::AlvarMarker marker = req.markers[i];
      //tf::Quaternion q(marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w);
      //tf::Matrix3x3 m(q);
      //double roll, pitch, yaw;
      //m.getRPY(roll, pitch, yaw);
      //ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
      int id = marker.id;
      int con = marker.confidence;
      geometry_msgs::Pose pose = marker.pose.pose;
      geometry_msgs::Point position = pose.position;
      //geometry_msgs::Quaternion quat=pose->orientation;//quaternions, four elements.
      double x = position.x;
      double y = position.y;
      double z = position.z;
      switch (id) {
      case 3:
        p.buildFrom(x, y, z);
        p.set_x(x);
        p.set_y(y);

        //p.set(z);
        // geometry_msgs::Point msg;
        // msg.x = dx;
        // msg.y = dy;
        // msg.z=0;
        // //Publish the message.
        // pub_p.publish(msg);
        //cout<<c_green<<"target Point:  " << x<<" "<<y<<" "<<z<<c_default<<endl;
        ready_signal4 = true;
        break;
      }
    }
  }
}

//inStr "0 0 0 0"
void moveRobotByQdot(cv::Mat qdot) {
  lock = true;
  std::vector < float > jnts;
  for (size_t i = 0; i < 7; i++) {
    if (i < dofNum) {
      float newJ = (float)(current_Joint_pose[i] + qdot.at < double > (i, 0));
      jnts.push_back(newJ);
    } else
      jnts.push_back(current_Joint_pose[i]);
  }

  mv_srv.request.joints = jnts;
  cout << "send to robot to the  position " << endl;
  //string_convertor::printOutStdVector(jnts);
  // cout << "Press any key to continue..." << endl;
  // getchar();
  Joint_move_client.call(mv_srv); ///////////////////////////////////////////////////
  //boost::this_thread::sleep( boost::posix_time::milliseconds(1000) );
  lock = false;
}

double get_error(vpFeaturePoint pc, vpFeaturePoint pd) //mode=0, orientation;  mode=1, position
{
  double xc = pc.get_x();
  double yc = pc.get_y();
  double zc = pc.get_Z();
  double xd = pd.get_x();
  double yd = pd.get_y();
  double zd = pd.get_Z();
  cv::Mat delta = (Mat_ < double > (2, 1) << xd - xc, yd - yc);
  return math_helper::FrobeniusNorm(delta);
}

double error_convrge_thres(int mode) {
  if (mode == 1) //position
    return 0.001;
  else
    return 0.0001;
}

double error_thres(int mode) {
  if (mode == 1)
    return 0.01;
  else
    return 0.01;
}

void ibvs_control() {
  double lamda = 0.2;
  double deltaT = 1;
  double error_thres = 0.001;
  double error_converge_thres = 0.0000001;
  vpServo task;
  vpFeaturePoint pd;
  pd.buildFrom(0, 0, 0.62); //desired point

  task.addFeature(p, pd);
  task.setServo(vpServo::EYEINHAND_CAMERA);
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
  task.setLambda(0.4);
  double error = get_error(p, pd);
  ibvs controller;
  double error_converge = 999;
  while (abs(error_converge) > error_converge_thres && abs(error) > error_thres) {
    vpColVector v(6); //velocity vector
    v = task.computeControlLaw(); //camera velocity expressed inthe camera frame
    cv::Mat T = pose::getTransformationE2B(current_trans, current_rot);
    cv::Mat V(6, 1, cv::DataType < double > ::type);
    for (int i = 0; i < 6; i++)
      V.at < double > (i, 0) = v[i];
    //match the camera velocity to end effector velocity using camera calibration
    //match the end effector velocity to joint velocity using jacobian.
    //cv::Mat qdot0 = controller.compute_qdot(currentJacobian, V , T);
    cv::Mat qdot = controller.compute_qdot(currentJacobian, V, current_trans, current_rot.t());
    qdot = qdot * deltaT;
    std::cout << c_yellow << "qdot" << endl << qdot * 57.2957795131 << c_default << endl;
    //normalize qdot.
    //cv::Mat qdot_normalize = pbvs::normalize_qdot(qdot,10);
    //std::cout<<c_yellow<<"delta q"<<endl<<qdot_normalize * 57.2957795131<<c_default<<endl;
    std::cout << c_green << "current_error: " << error << endl << "error by ibvs  " << task.getError() << endl << "error_converge: " << error_converge << c_default << endl;
    std::cout << c_red << "command moving robot! Press any key to continue..." << c_default << endl;
    getchar();
    //boost::this_thread::sleep( boost::posix_time::milliseconds(500) );
    moveRobotByQdot(qdot);

    //reset ready_signal
    ready_signal1 = false;
    ready_signal2 = false;
    ready_signal3 = false;
    while (!signal_ready()) {
      ros::spinOnce();
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    if (task.featureList.size() == 0)
      task.addFeature(p, pd);
    //update error
    std::cout << "current pose" << endl << p.get_x() << " " << p.get_y() << " " << p.get_Z() << endl;
    double error_p = get_error(p, pd);
    error_converge = error_p - error;
    error = error_p;
  }
}

//===========================MAIN FUNCTION START===========================

int main(int argc, char * argv[]) {

  double incremental = 0.1;
  ros::init(argc, argv, "ibvs");
  ros::NodeHandle nh;
  ros::Subscriber subP = nh.subscribe("/zeus/wam/pose", 1, wamPoseCallback);
  ros::Subscriber jacobian_sub = nh.subscribe("zeus/wam/jacobian", 1, wamToolJacobianCallback);
  ros::Subscriber wam_pos_sub = nh.subscribe("/zeus/wam/joint_states", 1, wamJointsCallback);
  Joint_move_client = nh.serviceClient < wam_srvs::JointMove > ("/zeus/wam/joint_move");

  ros::Subscriber sub = nh.subscribe("/ar_pose_marker", 1000, ar_pose_marker_callback);

  while (!signal_ready()) {
    ros::spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

  ibvs_control();

  ros::spin();
  //declaration of function
  ros::shutdown();
  return 0;
}
