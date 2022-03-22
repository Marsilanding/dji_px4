/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>


#include <sensor_msgs/Joy.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
/*Added*/
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/Thrust.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <uav_abstraction_layer/AttSetpoint.h>
#include <uav_abstraction_layer/AttRateSetpoint.h>


int ctrl_flag = 0;

mavros_msgs::State mavros_state_;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    mavros_state_ = *msg;
}

/*Callback de GPS*/
sensor_msgs::NavSatFix current_gps;
void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    current_gps = *msg;
}

/*Callback de posicion local*/
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

/*sensor_msgs::Joy joy_commandXY;
sensor_msgs::Joy joy_commandZYaw;*/
sensor_msgs::Joy joy_command;
std_msgs::Float64 joy_x;
std_msgs::Float64 joy_y;
std_msgs::Float64 joy_z;
std_msgs::Float64 joy_yaw;


/*void command_recXY(const sensor_msgs::Joy::ConstPtr& msg){
    joy_commandXY = *msg;
    joy_x.data = joy_commandXY.axes[0];
    joy_y.data = joy_commandXY.axes[1];
}

void command_recZYaw(const sensor_msgs::Joy::ConstPtr& msg){
    joy_commandZYaw = *msg;
    joy_z.data = joy_commandZYaw.axes[1];
    joy_yaw.data = joy_commandZYaw.axes[0];
}*/


/*Comando completo para caso real*/
void dji_enuposition_yaw(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command = *msg;
    joy_x.data = joy_command.axes[0];
    joy_y.data = joy_command.axes[1];
    joy_z.data = joy_command.axes[2];
    joy_yaw.data = joy_command.axes[3];
    ctrl_flag = 0;   
}

void dji_enuvelocity_yawrate(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command = *msg;
    joy_x.data = joy_command.axes[0];
    joy_y.data = joy_command.axes[1];
    joy_z.data = joy_command.axes[2];
    joy_yaw.data = joy_command.axes[3]; 
    ctrl_flag = 1;  
}

void dji_rollpitch_yawrate_z(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command = *msg;
    joy_x.data = joy_command.axes[0];
    joy_y.data = joy_command.axes[1];
    joy_z.data = joy_command.axes[2];
    joy_yaw.data = joy_command.axes[3]; 
    ctrl_flag = 2;  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber mavros_state = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix> /*Global Position*/
            ("mavros/global_position/global", 10, gps_cb);

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped> /*Local Position*/
            ("ual/pose", 10, pose_cb);


    /*Suscriptores para caso real*/
    ros::Subscriber dji_enuposition_yaw_sub = nh.subscribe<sensor_msgs::Joy>
        ("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10, dji_enuposition_yaw);

    ros::Subscriber dji_enuvelocity_yawrate_sub = nh.subscribe<sensor_msgs::Joy>
        ("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10,  dji_enuvelocity_yawrate);

    ros::Subscriber dji_rollpitch_yawrate_zpos_sub = nh.subscribe<sensor_msgs::Joy>
        ("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10, dji_rollpitch_yawrate_z);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/ual/set_pose", 10);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/ual/set_velocity", 10);

    ros::Publisher local_attitude_pub = nh.advertise<uav_abstraction_layer::AttSetpoint>
            ("/ual/set_attitude", 10);

    ros::Publisher local_attitude_rate_pub = nh.advertise<uav_abstraction_layer::AttSetpoint>
            ("/ual/set_attitude_rate", 10);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !mavros_state_.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /*Given wp*/
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;

    uav_abstraction_layer::AttSetpoint attitude;
    attitude.orientation.x = 0;
    attitude.orientation.y = 0;
    attitude.orientation.z = 0;

    uav_abstraction_layer::AttRateSetpoint attitude_rate;
    attitude_rate.body_rate.x = 0;
    attitude_rate.body_rate.y = 0;
    attitude_rate.body_rate.z = 0;


    ros::Time last_request = ros::Time::now();


    while(ros::ok()){

        if(ctrl_flag == 0){ /*para /dji_sdk/flight_control_setpoint_ENUposition_yaw*/
            pose.pose.position.x = current_pose.pose.position.x + joy_x.data; /*X offset*/
            pose.pose.position.y = current_pose.pose.position.y; + joy_y.data; /*Y offset*/
            pose.pose.position.z = current_pose.pose.position.z; + joy_z.data; /*height*/
            attitude.orientation.z = current_pose.pose.orientation.z + joy_yaw.data; /*yaw angle*/
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
            local_attitude_pub.publish(attitude);
        }

        else if(ctrl_flag == 1) { /*para /dji_sdk/flight_control_setpoint_ENUvelocity_yawrate*/
            twist.twist.linear.x = joy_x.data; /*velocidad en x*/
            twist.twist.linear.y = joy_y.data; /*velocidad en y*/
            twist.twist.linear.z = joy_z.data; /*velocidad en z*/
            attitude_rate.body_rate.z = joy_yaw.data; /*yawrate*/
            local_vel_pub.publish(twist);
            ros::spinOnce();
            rate.sleep();
            local_attitude_rate_pub.publish(attitude);
        }

        else{
            attitude.orientation.x = current_pose.pose.orientation.x + joy_x.data; /*roll*/
            attitude.orientation.y = current_pose.pose.orientation.y + joy_y.data; /*pitch*/
            attitude.orientation.z = current_pose.pose.position.z + joy_z.data;    /*yaw*/
            attitude_rate.body_rate.z = joy_yaw.data;  /*yaw rate*/
            local_attitude_pub.publish(attitude);
            ros::spinOnce();
            rate.sleep();
            local_attitude_rate_pub.publish(attitude_rate);
            }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
