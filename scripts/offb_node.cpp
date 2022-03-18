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


int ctrl_flag = 3;

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

sensor_msgs::Joy joy_command;
std_msgs::Float64 joy_x;
std_msgs::Float64 joy_y;

void command_rec(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command = *msg;
    joy_x.data = joy_command.axes[0];
    joy_y.data = joy_command.axes[1];
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
            ("mavros/local_position/pose", 10, pose_cb);

    ros::Subscriber sp_command_sub = nh.subscribe<sensor_msgs::Joy> 
            ("/joy", 10, command_rec);        

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::Publisher local_attitude_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 10);

    std::string set_mode_srv = "mavros/set_mode"; /*de UAL*/
    ros::ServiceClient flight_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>(set_mode_srv.c_str()); /*de UAL*/

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
    pose.pose.position.z = 2;
    pose.pose.orientation.z = 0;

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0;
    twist.twist.linear.y = 0;
    twist.twist.linear.z = 0;
    twist.twist.angular.z = 0;

    geometry_msgs::PoseStamped orientation;
    orientation.pose.orientation.x = 0;
    orientation.pose.orientation.y = 0;
    orientation.pose.orientation.z = 0;

    ros::Time last_request = ros::Time::now();

    int flag = 0;

    mavros_msgs::SetMode flight_mode_service;
    flight_mode_service.request.base_mode = 0;
    flight_mode_service.request.custom_mode = "OFFBOARD";
    // Set mode: unabortable?

    while(ros::ok()){

        while (mavros_state_.mode != "OFFBOARD") {
            if (!flight_mode_client_.call(flight_mode_service)) {
                ROS_ERROR("Error in set flight mode [%s] service calling!", "OFFBOARD");
            }
            ros::spinOnce();
            #ifdef MAVROS_VERSION_BELOW_0_20_0
                ROS_INFO("Set flight mode [%s] response.success = %s", "OFFBOARD", \
                    flight_mode_service.response.success ? "true" : "false");
            #else
                ROS_INFO("Set flight mode [%s] response.success = %s", "OFFBOARD", \
                    flight_mode_service.response.mode_sent ? "true" : "false");
            #endif
            ROS_INFO("Trying to set [%s] mode; mavros_state_.mode = [%s]", "OFFBOARD", mavros_state_.mode.c_str());
        }

        if(ctrl_flag == 0){
            pose.pose.position.x = pose.pose.position.x + joy_x.data; /*joy_command.axes[0]*/
            pose.pose.position.y = pose.pose.position.y + joy_y.data;
            pose.pose.position.z = 2;  
            local_pos_pub.publish(pose);
        }

        else if(ctrl_flag == 1) {
            twist.twist.linear.x = joy_x.data; /*joy_command.axes[0]*/
            twist.twist.linear.y = joy_y.data;
            twist.twist.linear.z = 0.1;
            local_vel_pub.publish(twist);
        }

        else{
            orientation.pose.orientation.x = orientation.pose.orientation.x + joy_x.data; /*joy_command.axes[0]*/
            orientation.pose.orientation.y = orientation.pose.orientation.y + joy_y.data;
            pose.pose.position.z = 2;    
            local_attitude_pub.publish(orientation);
            ros::spinOnce();
            rate.sleep();
            local_pos_pub.publish(pose);
            }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


/*void BackendMavros::setFlightMode(const std::string& _flight_mode) {
    mavros_msgs::SetMode flight_mode_service;
    flight_mode_service.request.base_mode = 0;
    flight_mode_service.request.custom_mode = _flight_mode;
    // Set mode: unabortable?
    while (mavros_state_.mode != _flight_mode && ros::ok()) {
        if (!flight_mode_client_.call(flight_mode_service)) {
            ROS_ERROR("Error in set flight mode [%s] service calling!", _flight_mode.c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
#ifdef MAVROS_VERSION_BELOW_0_20_0
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.success ? "true" : "false");
#else
        ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
            flight_mode_service.response.mode_sent ? "true" : "false");
#endif
        ROS_INFO("Trying to set [%s] mode; mavros_state_.mode = [%s]", _flight_mode.c_str(), mavros_state_.mode.c_str());
    }
}*/