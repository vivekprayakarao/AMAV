#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/Twist.h"


int main(int argc, char **argv)
{
   ros::init(argc, argv, "comm");
   ros::NodeHandle n;

   ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
   ros::Rate loop_rate(10);

   geometry_msgs::Twist msg;

   
       for(int i=0;i<20;i++){
         
       msg.linear.x = 0.0;
       msg.angular.z = -0.5;
       chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();   }
	for(int i=0;i<100;i++){
         
       msg.linear.x = 1.0;
       msg.angular.z = 0.0;
	chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();       
}

       
       
   return 0;
}

