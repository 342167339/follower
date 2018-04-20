//lead-follow控制器，包括lead-follow阵型跟踪控制器和轨迹追踪控制器
//订阅leader的odom话题，根据给定条件利用阵型跟踪控制器计算出虚拟参考机器人的odom信息
//根据虚拟参考机器人的odom信息，利用tracking控制器计算follower的控制率并发布
//阵型控制器表现良好，tracking控制器效果不好

#include <ros/ros.h>  
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <math.h>
#include <tf/tf.h>

#define pi 3.1415926
using namespace Eigen;

class LeaderAndFollow  
{  
public:  
  LeaderAndFollow()  
  {  
    pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel_follower", 1);  
    // pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);  
    pub_odom = n_.advertise<nav_msgs::Odometry>("/reference_odom", 1);
    sub_A = n_.subscribe("/follower_odom", 1, &LeaderAndFollow::A_callback, this);
    sub_B = n_.subscribe("/odom", 1, &LeaderAndFollow::B_callback, this);
    last = ros::Time::now();
    ld = 1; phid = 0.75 * pi;
    d = 0.1;
    k1 = 1; k2 = 1; kx = 1; ktheta = 1;
    beta = 0; phi = 0;
    Br.header.stamp = ros::Time::now();
    Br.header.frame_id = "/odom";
  }

  void A_callback(const nav_msgs::Odometry input);
  void B_callback(const nav_msgs::Odometry input);
  double CalculatePhi(const nav_msgs::Odometry A);
  void UpdatePosition();
  void GetReferenceVelocity(const nav_msgs::Odometry A);
  void GetFollowerVelocity();
  double QuaternionToRPY(geometry_msgs::Quaternion q);//输入四元数，返回绕z轴旋转角度，因为只有z轴旋转
  geometry_msgs::Quaternion RPYToQuaternion(double theta);//输入转角，返回四元数
  double DealWithEtheta(double etheta);//处理etheta

private:  
  ros::NodeHandle n_;   
  ros::Publisher pub_;
  ros::Publisher pub_odom;  
  ros::Subscriber sub_A;  
  ros::Subscriber sub_B;
  nav_msgs::Odometry B;
  nav_msgs::Odometry Br;
  nav_msgs::Odometry Br_last;
  geometry_msgs::Twist output;
  ros::Time last;
  double ld, phid;
  double d;
  double k1, k2, kx, ktheta;
  double beta, phi;
};

//当收到leader位置更新时，调用两个控制器计算follower的控制率
void LeaderAndFollow::A_callback(const nav_msgs::Odometry A)  
{  
  ros::Rate rate(10.0);
  GetReferenceVelocity(A);//包括航迹推演
  GetFollowerVelocity();
  output.linear.x = B.twist.twist.linear.x;
  output.angular.z = B.twist.twist.angular.z;
  pub_.publish(output);      
  Br.header.stamp = ros::Time::now();
  pub_odom.publish(Br);
  Br_last = Br;
  last = ros::Time::now(); 
  rate.sleep();
}  

//将最新的follower的位置信息保存
void LeaderAndFollow::B_callback(const nav_msgs::Odometry input){
  B = input;
}

//阵型保持控制器，根据leader的控制率计算出虚拟参考机器人的控制率
void LeaderAndFollow::GetReferenceVelocity(const nav_msgs::Odometry A){
  beta = QuaternionToRPY(A.pose.pose.orientation) - QuaternionToRPY(Br_last.pose.pose.orientation);
  phi = CalculatePhi(A);
  double l = sqrt( pow( A.pose.pose.position.x - Br_last.pose.pose.position.x, 2 ) + 
              pow( A.pose.pose.position.y - Br_last.pose.pose.position.y, 2 ) );
  MatrixXd G(2,2), F(2,2), P(2,1), UA(2,1), UBR(2,1);
  G << cos(beta + phi), d * sin(beta + phi),
        -sin(beta + phi)/l, d * cos(beta + phi)/l;
  F << -cos(phi), 0, sin(phi)/l, -1;
  P << k1 * ( ld - l), k2 * (phid - phi);
  UA << A.twist.twist.linear.x, A.twist.twist.angular.z;
  UBR << 0, 0;
  UBR = G.inverse() * (P - F * UA);
  Br.twist.twist.linear.x = UBR(0,0);
  Br.twist.twist.angular.z = UBR(1,0);
  UpdatePosition();
}

//tracking控制器，根据虚拟参考机器人的控制率计算follower的控制率
void LeaderAndFollow::GetFollowerVelocity(){
  double etheta_tmp = QuaternionToRPY(Br.pose.pose.orientation) - QuaternionToRPY(B.pose.pose.orientation);
  //对etheta取余，后面有个除以etheta
  
  double etheta;
  etheta = DealWithEtheta(etheta_tmp);//不能double etheta = DealWithEtheta(etheta_tmp);会出错
  //ROS_INFO("theta = %f", etheta);
  double ex = Br.pose.pose.position.x - B.pose.pose.position.x;
  double ey = Br.pose.pose.position.y - B.pose.pose.position.y;
  B.twist.twist.linear.x = Br.twist.twist.linear.x * cos(etheta) 
                          + kx * ex;
  B.twist.twist.angular.z = Br.twist.twist.angular.z 
                          + ktheta * etheta 
                          + Br.twist.twist.linear.x * ey * sin(etheta) / etheta;
}

//计算两个机器人朝向的夹角
double LeaderAndFollow::CalculatePhi(const nav_msgs::Odometry A){
  double theta = atan2( Br_last.pose.pose.position.y - A.pose.pose.position.y, 
                        Br_last.pose.pose.position.x - A.pose.pose.position.x )
                -atan2( sin( QuaternionToRPY( A.pose.pose.orientation) ), 
                        cos( QuaternionToRPY( A.pose.pose.orientation) ) );
  if(theta<0){
    phi = theta+2*pi;
  }else{
    phi = theta;
  }  
  return phi;
}

//更新虚拟参考机器人的位置信息
void LeaderAndFollow::UpdatePosition(){
  ros::Time now = ros::Time::now();
  //double dt = ( now - last ).toSec();
  double dt = 0.1;
  double theta_tmp = Br.twist.twist.angular.z * dt;
  double theta_last = QuaternionToRPY(Br_last.pose.pose.orientation);
  //ROS_INFO("theta = %f", theta_last*180);
  // Br.pose.pose.position.x = Br_last.pose.pose.position.x 
  //     + Br.twist.twist.linear.x * dt * cos( theta_last + theta_tmp/2);
  // Br.pose.pose.position.y = Br_last.pose.pose.position.y 
  //     + Br.twist.twist.linear.x * dt * sin( theta_last + theta_tmp/2);
  // Br.pose.pose.orientation = RPYToQuaternion( theta_last + theta_tmp);
  Br.pose.pose.position.x = Br_last.pose.pose.position.x 
      + Br.twist.twist.linear.x * dt * cos( theta_last );
  Br.pose.pose.position.y = Br_last.pose.pose.position.y 
      + Br.twist.twist.linear.x * dt * sin( theta_last );
  Br.pose.pose.orientation = RPYToQuaternion( theta_last + theta_tmp);
}

//将四元数转换为rpy欧拉角中绕z轴旋转的yaw角度
double LeaderAndFollow::QuaternionToRPY(geometry_msgs::Quaternion q){
  double x = q.x;
  double y = q.y;
  double z = q.z;
  double w = q.w;
  //用tf的变换公式
  //tf::Quaternion qr(0,0,Br.pose.pose.orientation.z, Br.pose.pose.orientation.w);
  //tf::Quaternion q(0,0,B.pose.pose.orientation.z, B.pose.pose.orientation.w);
  //tf::Vector3 vr =  qr.getAxis();
  //tf::Vector3 v =  q.getAxis();

  //直接用公式
  double ang = atan2( 2*( w*z + x*y ), 1-2*( y*y + z*z) );
  //ROS_INFO("ang = %f", ang);
  return ang;
}

//将rpy欧拉角中绕z轴旋转的yaw角度转换为四元数
geometry_msgs::Quaternion LeaderAndFollow::RPYToQuaternion(double theta){
  geometry_msgs::Quaternion q;
  q.x = 0;
  q.y = 0;
  q.z = sin( theta/2 );
  q.w = cos( theta/2 );

  return q;
}

//处理phi，将绝对值大于2pi的减去周期2pi
double LeaderAndFollow::DealWithEtheta(double etheta){
  double etheta_deal;
//绝对值小于2pi不处理
  if( abs( etheta ) > ( 2 * pi ) ){//绝对值大于2pi
    int a = abs(etheta) / (2 * pi);
    if(etheta>0){
        etheta_deal = etheta - a * 2 * pi;//大于零时减去2pi的整数倍
    }else{
        etheta_deal = etheta + a * 2 * pi;
    }
  }
  if(etheta == 0){
    etheta_deal = 2*pi;
  }
  
  //ROS_INFO("theta_hanshu = %f", etheta_deal);
  return etheta_deal;
}


int main(int argc, char **argv)  
{  
  ros::init(argc, argv, "leader_follower"); 
  ros::NodeHandle n; 
  LeaderAndFollow LAFObject;  
  //ros::Rate rate(10.0);
  //ros::Timer timer1 = n.createTimer(ros::Duration(0.1), LAFObject.A_callback());
  //ros::Timer timer2 = n.createTimer(ros::Duration(0.1), LAFObject.B_callback());
  
  ros::spin();  
  
  return 0;  
}  
