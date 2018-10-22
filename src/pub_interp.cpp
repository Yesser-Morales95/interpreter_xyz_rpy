#include "ros/ros.h"
#include "std_msgs/String.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>



//double returnValue()
//{
//  return joint;
////  jointz = susc;
//}

class SubscriberAndPublish {
public:
  SubscriberAndPublish(){
    joint_sub = nh.subscribe("/set_joint_trajectory",1,&SubscriberAndPublish::trajectoryCallback, this);
    joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("set_joint_trajectory_delay", 1);
    while(ros::ok())
       {
           //ROS_WARN("In Loop.");
           ros::spin();
       }
  }
  void trajectoryCallback(const trajectory_msgs::JointTrajectory &msg);
  trajectory_msgs::JointTrajectory msg1;
private:
    ros::Subscriber joint_sub;
    ros::Publisher joint_pub;
    ros::NodeHandle nh;

};

//SubscriberAndPublish::SubscriberAndPublish(){
//  joint_sub = nh.subscribe("/set_joint_trajectory",1,&SubscriberAndPublish::trajectoryCallback, this);
//  joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("set_joint_trajectory_delay", 1);
//}
void SubscriberAndPublish::trajectoryCallback(const trajectory_msgs::JointTrajectory &msg)
{
                  double jointx, joint;
                  std::cout <<msg<<std::endl;

                  jointx = msg.points[0].positions[0];
                  for (double i = 0; i<20; i++){

                  joint= (i/20)*jointx;
                  boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                  std::cout <<joint<<std::endl;
                  msg1.points.resize(1);
                  msg1.points[0].positions.resize(1);
//                  msg1.points[0].positions[0].push_back(joint); //array [i]
                  msg1.points[0].positions[0]=joint;
                  msg1.points[0].time_from_start = ros::Duration(0.1);
                  msg1.header.frame_id = "my_lab_uni_delay";
//                  msg1.header.stamp = ros::Time::now();
                  joint_pub.publish(msg1) ;
                    }//end FOR

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_interp");
  SubscriberAndPublish SaPObject;
//  ros::Rate	loop_rate(10);

//      while	(ros::ok())
//        {
////        joint=joint + (jointx/10);
////             do{
////SaPObject.joint_sub;
//                  ros::spinOnce();
//                  loop_rate.sleep();
////                  flag++;
////                  }  while(flag!=20);

//      } //end While


  return 0;
}
