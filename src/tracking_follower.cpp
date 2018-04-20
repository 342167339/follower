//点追踪控制器
//接受follower_odom是leader的odom经过坐标变换得到的，如leader的左下角45度1米
//计算出follower的速度，发布
//效果不好

#include <ros/ros.h>  
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <math.h>

#define pi 3.1415926;
using namespace Eigen;

class LeaderAndFollow  
{  
public:  
  LeaderAndFollow()  
  {  
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel_follower", 1);  
    sub_A = n_.subscribe("/follower_odom", 1, &LeaderAndFollow::A_callback, this);
    sub_B = n_.subscribe("/odom", 1, &LeaderAndFollow::B_callback, this);
    kx = 0.5; ktheta = 4;
  }
  
  void A_callback(const nav_msgs::Odometry input);
  void B_callback(const nav_msgs::Odometry input);
  double QuaternionToRPY(geometry_msgs::Quaternion q);
  //geometry_msgs::Twist GetFollowerVelocity(const nav_msgs::Odometry A);

private:  
  ros::NodeHandle n_;   
  ros::Publisher pub_;  
  ros::Subscriber sub_A;  
  ros::Subscriber sub_B;
  nav_msgs::Odometry B;
  double last;
  double ld, phid;
  double d;
  double k1, k2, kx, ktheta;
  double beta, phi;
};

void LeaderAndFollow::A_callback(const nav_msgs::Odometry A)  
{  
  ros::Rate rate(10.0);
  //geometry_msgs::Twist output;
  //ROS_INFO("leader:x = %f, follower:x = %f, diff = %f", 
    //A.pose.pose.position.x,B.pose.pose.position.x,A.pose.pose.position.x - B.pose.pose.position.x);
  // output.linear.x = 0.5 * sqrt(pow(A.pose.pose.position.x - B.pose.pose.position.x, 2) +
  //                                 pow(A.pose.pose.position.y - B.pose.pose.position.y, 2)); 
  // output.angular.z = 10 * atan2( A.pose.pose.position.y - B.pose.pose.position.y,
  //                                   A.pose.pose.position.x - B.pose.pose.position.x);
  //pub_.publish(output); 

  double etheta = QuaternionToRPY(A.pose.pose.orientation) - QuaternionToRPY(B.pose.pose.orientation);
  double ex = A.pose.pose.position.x - B.pose.pose.position.x;
  double ey = A.pose.pose.position.y - B.pose.pose.position.y;
  B.twist.twist.linear.x = A.twist.twist.linear.x * cos(etheta) + kx * ex;
  B.twist.twist.angular.z = A.twist.twist.angular.z + ktheta * etheta 
                            + A.twist.twist.linear.x * ey * sin(etheta) / etheta;
  pub_.publish(B.twist.twist);
  
  rate.sleep();
}  

void LeaderAndFollow::B_callback(const nav_msgs::Odometry input){
  B = input;
}

double LeaderAndFollow::QuaternionToRPY(geometry_msgs::Quaternion q){
  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;

  double ang = atan2( 2*( w*z + x*y ), 1-2*( y*y + z*z) );
  //ROS_INFO("ang = %f", ang);
  return ang;
}

int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "leader_follower"); 
  ros::NodeHandle n; 
  LeaderAndFollow LAFObject;  

  ros::spin();  
  
  return 0;  
}  
