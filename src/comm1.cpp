#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/Twist.h"

void draw_line(float a,float b,int n,int m);

int main(int argc, char **argv)
{
   ros::init(argc, argv, "comm1");
   

   draw_line(1.0,1.5,10,20);
   draw_line(1.0,-4.5,10,20);
   draw_line(1.0,-3.0,10,20);
   //draw_line(1.0,-4.5,10,20);   
   return 0;
}

void draw_line(float a,float b,int p,int m)
{
   ros::NodeHandle n;
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
   ros::Rate loop_rate(10);

   geometry_msgs::Twist msg;

   
       for(int i=0;i<p;i++)
	{  
       msg.linear.x = 0;
       msg.angular.z = b;
       chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();   
	}
	
	for(int i=0;i<m;i++)
	{  
       msg.linear.x = a;
       msg.angular.z = 0;
       chatter_pub.publish(msg);
       ros::spinOnce();
       loop_rate.sleep();   
	}
}
  
