/**
 * @file dji_client.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>



#include <sensor_msgs/Joy.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
/*Added*/
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/Thrust.h>
#include <sensor_msgs/NavSatFix.h>

#include <nav_msgs/Odometry.h>

#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <uav_abstraction_layer/AttSetpoint.h>
#include <uav_abstraction_layer/AttRateSetpoint.h>
#include <uav_abstraction_layer/SetHome.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/Land.h>

#include <dji_px4/DroneTaskControl.h>
#include <dji_px4/SDKControlAuthority.h>


sensor_msgs::Joy joy_command;

/*Comando completo para caso real*/


/*void joy_XY_cb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_commandXY = *msg;
    joy_command.axes[0] = joy_commandXY.axes[0];
    joy_command.axes[1] = joy_commandXY.axes[1];
}

void joy_ZYaw_cb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_commandZYaw = *msg;
    joy_command.axes[2] = joy_commandZYaw.axes[0];
    joy_command.axes[3] = joy_commandZYaw.axes[1];
}*/

void joy_cb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dji_client");
    ros::NodeHandle nh;

    nh.setParam("/dji_client/control_mode", 5);
    nh.setParam("/dji_client/takeoff_height", 2.0);
    nh.setParam("/dji_client/blocking", 1);
    nh.setParam("/dji_client/control_authority", 0);


    /*Suscriptores para caso real*/

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>
        ("/dji_commands", 10, joy_cb);

    /*Publicadores que envian comandos*/

    ros::Publisher dji_generic_pub = nh.advertise<sensor_msgs::Joy>
            ("/dji_sdk/flight_control_setpoint_generic", 10);

    ros::Publisher dji_enuposition_yaw_pub = nh.advertise<sensor_msgs::Joy>
            ("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

    ros::Publisher dji_enuvelocity_yawrate_pub = nh.advertise<sensor_msgs::Joy>
            ("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);

    ros::Publisher dji_rollpitch_yawrate_zpos_pub = nh.advertise<sensor_msgs::Joy>
            ("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);

    ros::ServiceClient dji_authority_client = nh.serviceClient<dji_px4::SDKControlAuthority>
            ("/dji_sdk/sdk_control_authority");

    ros::ServiceClient dji_takeoff_client = nh.serviceClient<dji_px4::DroneTaskControl>
            ("/dji_sdk/drone_task_control");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    int control_param = 0;

    while(ros::ok()){

        ros::param::get("/dji_client/control_mode", control_param);

        if(control_param == 0) dji_generic_pub.publish(joy_command);
        else if(control_param == 1) dji_enuposition_yaw_pub.publish(joy_command);
        else if(control_param == 2) dji_enuvelocity_yawrate_pub.publish(joy_command);
        else if(control_param == 3) dji_rollpitch_yawrate_zpos_pub.publish(joy_command);

        ros::spinOnce();
        rate.sleep();  
    }

    return 0;
}
