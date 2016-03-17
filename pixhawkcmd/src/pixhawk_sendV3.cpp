#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>

//parameters
geometry_msgs::PoseStamped cmd_att;
std_msgs::Float64 cmd_thr;
int count = 1;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;

    ROS_INFO("command sent");
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "pixhawk_send");
   ros::NodeHandle nh;

   ros::Publisher pub_att = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_attitude/attitude",10);
   ros::Publisher pub_thr = nh.advertise<std_msgs::Float64>
            ("/mavros/setpoint_attitude/att_throttle", 10);
   ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
   ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
   ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

   ros::Rate rate(100);


   //wait for FCU connection
   while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
   }

    //``````````````MODIFY THESE!!!!``````````````````````
    double v[3]={1.0, 0.0, 0.0};
    double throttle_val = 0.3;                                  
    double v_norm=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    double theta=0.1;                                     
    //````````````````````````````````````````````````````

       cmd_att.header.stamp = ros::Time::now();
       cmd_att.header.seq=count;
       cmd_att.header.frame_id = 1;
       //cmd_att.pose.position.x = 0.0;//0.001*some_object.position_x;
       //cmd_att.pose.position.y = 0.0;//0.001*some_object.position_y;
       //cmd_att.pose.position.z = 0.0;//0.001*some_object.position_z;

       cmd_att.pose.orientation.x = quaternion[0];
       cmd_att.pose.orientation.y = quaternion[1];
       cmd_att.pose.orientation.z = quaternion[2];
       cmd_att.pose.orientation.w = quaternion[3];

       //Create throttle command message
       cmd_thr.data = throttle_val;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pub_att.publish(cmd_att);
        pub_thr.publish(cmd_thr);
        ros::spinOnce();
        rate.sleep();
    }

    //OFFBOARD mode code
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //ARM the motors
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
        //if( current_state.mode != "POSCTL" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        pub_att.publish(cmd_att);
        pub_thr.publish(cmd_thr);

        ros::spinOnce();
        count++;
        theta=0.3*sin(count/300.0);
        rate.sleep();
    }


   return 0;
}
