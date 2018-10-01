#include "ibvs.h"

ibvs::ibvs()
{
  lamda=0.1;
  //normalizeJointScale = 0.01745329251;//1 degree
}
ibvs::~ibvs(){}
ibvs::ibvs(double _lamda)
{ 
  lamda=_lamda;
  //normalizeJointScale=_normalizeJointScale*0.01745329251;
}

//T, is the current transformation from end effector frame to robot base.
//_J, is the jacobian defined as from joint space to linear velocity in robot base frame
//vc camera velocity expressed in camera's frame
cv::Mat ibvs::compute_qdot(cv::Mat _J, cv::Mat V, cv::Mat _T)
{
  //update J, and currentPose
  //in this case, we treat camera frame the same as the end effector frame
  //but only transform it to be expressed in robot base frame
  cv::Mat Ve_b = pose::getVelocityTransform(_T);
 
  cout<<"V desired"<<endl<<V<<endl;
  //do velocity transformation
  cv::Mat V_eb = Ve_b * V;
  cout<<"V_eb"<<endl<<V_eb<<endl;
  //solve persudo invers of jacobian
  cv::Mat inv_J = math_helper::pinv(_J);
  cout<<"inv_J"<<endl<<inv_J<<endl;
  cv::Mat q_dot = inv_J * V_eb;
  cout<<"q_dot"<<endl<<q_dot<<endl;
  if(math_helper::FrobeniusNorm(inv_J)>100 )//in case of singularity
    for(size_t i=0;i<q_dot.rows;i++)
      q_dot.at<double>(i,0)=0.01;
  return q_dot;
  //you need to normalize delta_q, when you perform delta_q = q_dot * delta_t in your main controller
  //after that, you can send it to the robot.
}

cv::Mat ibvs::compute_qdot(cv::Mat _J, cv::Mat V, cv::Mat trans, cv::Mat R)
{
  //update J, and currentPose
  //in this case, we treat camera frame the same as the end effector frame
  //but only transform it to be expressed in robot base frame
  cout<<"V desired"<<endl<<V<<endl;
  cv::Mat V_l = V.rowRange(0,3).col(0);
  cv::Mat V_w = V.rowRange(3,6).col(0);  
  
  //do velocity transformation
  cv::Mat V_eb(6, 1,cv::DataType<double>::type);
  cv::Mat rwx = math_helper::skewFromVect(R*V_w);
  cv::Mat v_eb_l = R * V_l - rwx*trans;
  cv::Mat v_eb_w = R * V_w;
  v_eb_l.copyTo(V_eb.rowRange(0,3).col(0));
  v_eb_w.copyTo(V_eb.rowRange(3,6).col(0));
  cout<<"V_eb"<<endl<<V_eb<<endl;
  //solve persudo invers of jacobian
  cv::Mat inv_J = math_helper::pinv(_J);
  cout<<"inv_J"<<endl<<inv_J<<endl;
  cv::Mat q_dot = inv_J * V_eb;
  cout<<"q_dot"<<endl<<q_dot<<endl;
  if(math_helper::FrobeniusNorm(inv_J)>100 )//in case of singularity
    for(size_t i=0;i<q_dot.rows;i++)
      q_dot.at<double>(i,0)=0.01;
  return q_dot;
}

//normalize the joint delta value for safety
//_normalizeJointScale : unit degrees
cv::Mat ibvs::normalize_qdot(cv::Mat q_dot, double _normalizeJointScale)
{
  double normalizeJointScale=_normalizeJointScale*0.01745329251;
  double norm = math_helper::FrobeniusNorm(q_dot);
  if(norm != 0)
    return q_dot * normalizeJointScale / norm;
  else
    return q_dot;
}


//normalize the joint delta value for safety, using default scale value, which is 1 degree
cv::Mat ibvs::normalize_qdot(cv::Mat q_dot)
{
  double normalizeJointScale=0.01745329251;
  double norm = math_helper::FrobeniusNorm(q_dot);
  if(norm != 0)
    return q_dot * normalizeJointScale / norm;
  else
    return q_dot;
}

bool ibvs::check_qdot(cv::Mat qdot, double threshold)
{
  double thres = threshold*0.01745329251;
  for(size_t i=0;i<qdot.rows;i++)
    if(abs(qdot.at<double>(i,0))>thres)
        return false;
  return true;
}

//return relative pose
pose ibvs::getRelativePose(cv::Mat trans_e2b, cv::Mat rot_b2e, cv::Mat trans_ed2b, cv::Mat rot_b2ed)
{
  cv::Mat ed_e_B = trans_e2b - trans_ed2b;
  cv::Mat trans_e2ed = rot_b2ed * ed_e_B;
  cv::Mat rot_e2ed = rot_b2ed * rot_b2e.t();
  pose p(trans_e2ed, rot_e2ed, 0);
  //cout<<"relativePost: "<<endl<<"trans:"<<endl<<p.get_t()<<endl<<"rot:"<<endl<<p.get_R()<<endl;
  return p;
}
