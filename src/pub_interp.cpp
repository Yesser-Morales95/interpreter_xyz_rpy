#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <array>


class SubscriberAndPublish {
public:
  SubscriberAndPublish(){
    joint_sub = nh.subscribe("/set_joint_trajectory",1,&SubscriberAndPublish::trajectoryCallback, this);
    joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("set_joint_trajectory_delay", 10);
//    joint_pub_sub = nh.advertise<trajectory_msgs::JointTrajectory>("set_joint_trajectory", 10);

    msgpub.points.resize(1);
    msgpub.points[0].positions.resize(6);
    while(ros::ok())
       {
           //ROS_WARN("In Loop.");
           ros::spin();
       }

  }

  void trajectoryCallback(const trajectory_msgs::JointTrajectory &msg);

  trajectory_msgs::JointTrajectory msg1, msgpub;
  std::vector<double> jointvaluesOLD = {0.0,0.0,0.0,0.0,0.0,0.0};
private:
    ros::Subscriber joint_sub;
    ros::Publisher joint_pub;
//    ros::Publisher joint_pub_sub;
    ros::NodeHandle nh;

};

//SubscriberAndPublish::SubscriberAndPublish(){
//  joint_sub = nh.subscribe("/set_joint_trajectory",1,&SubscriberAndPublish::trajectoryCallback, this);
//  joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("set_joint_trajectory_delay", 1);
//}

void SubscriberAndPublish::trajectoryCallback(const trajectory_msgs::JointTrajectory &msg) // toma el ultimo valor
{

//                 jointvaluesOLD(6);
                  int it=0;
                  msg1.points.resize(1);
                  msg1.points[0].positions.resize(6);
                  double jointx, joint;
                  std::vector<double> jointvalues(6);
                  std::vector<double> velvalues(6);
                  std::vector<double> jointvaluesminus(6);
                  std::cout <<msg<<std::endl;
                  it = msg.points.size();
//                  msg.joint_names.pu
                  for (int m=1; m<it; m++){

                  for (int i = 0; i < 6; i++) {
                    jointvalues[i] = msg.points[m].positions[i];
                    velvalues  [i] = msg.points[m].velocities[i];
//                    msgpub.points[0].positions[i] = msg.points[0].positions[i];
                    jointvaluesminus[i] = jointvaluesOLD[i] - jointvalues[i];
                  }


                  for (double j = 0; j<=20; j++){
//                  joint= (i/20)*jointx;
                 for (int k = 0; k < 6; k++) {
                  msg1.points[0].positions[k] =jointvaluesOLD[k] + (((-1)*jointvaluesminus[k])*(j/20)); //array [i]
                  boost::this_thread::sleep(boost::posix_time::milliseconds(101 - velvalues  [k]));

                 }
                  msg1.points[0].time_from_start = ros::Duration(0.1);
                  msg1.header.frame_id = "my_lab_uni_delay";
//                  msg1.header.stamp = ros::Time::now();
                  joint_pub.publish(msg1) ;
                    }//end FOR

                   for (int l = 0; l < 6; l++) {
                  jointvaluesOLD[l]=jointvalues[l];
                   }

                                     } //for Points

//                   if (msg.points[0].positions[7]==0.0){

//                     for (int i = 0; i < 6; i++) {

//                       msgpub.points[0].positions[i] = msg.points[0].positions[i];
//                     }


//                     joint_pub_sub.publish(msgpub) ;

//                   }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_interp");
  SubscriberAndPublish SaPObject;
  return 0;
}
