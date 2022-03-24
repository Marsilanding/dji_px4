/**
 * @file offb_node.cpp
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
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/GlobalPositionTarget.h>

#include <uav_abstraction_layer/AttSetpoint.h>
#include <uav_abstraction_layer/AttRateSetpoint.h>
#include <uav_abstraction_layer/SetHome.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <uav_abstraction_layer/Land.h>

#include <dji_px4/DroneTaskControl.h>



int ctrl_flag = 4;
double takeoff_height = 2.0;
bool blocking = 1;

int dji_task = 0; /*tarea solicitada por el servicio drone_task_control de DJI*/

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
sensor_msgs::Joy joy_command_rec;
std_msgs::Float64 joy_x_rec;
std_msgs::Float64 joy_y_rec;
std_msgs::Float64 joy_z_rec;
std_msgs::Float64 joy_yaw_rec;
std_msgs::Float64 flag_msg; /*flag de /dji_sdk/flight_control_setpoint_generic*/
int flag = flag_msg.data; /*flag en entero*/



/*Comando completo para caso real*/

void dji_setpoint_generic(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command_rec = *msg;
    joy_x_rec.data = joy_command_rec.axes[0];
    joy_y_rec.data = joy_command_rec.axes[1];
    joy_z_rec.data = joy_command_rec.axes[2];
    joy_yaw_rec.data = joy_command_rec.axes[3];
    flag_msg.data = joy_command_rec.axes[4]; /*guardo valor del flag*/
    flag = (int)flag_msg.data;
    ctrl_flag = 0;   
}

void dji_enuposition_yaw(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command_rec = *msg;
    joy_x_rec.data = joy_command_rec.axes[0];
    joy_y_rec.data = joy_command_rec.axes[1];
    joy_z_rec.data = joy_command_rec.axes[2];
    joy_yaw_rec.data = joy_command_rec.axes[3];
    ctrl_flag = 1;   
}

void dji_enuvelocity_yawrate(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command_rec = *msg;
    joy_x_rec.data = joy_command_rec.axes[0];
    joy_y_rec.data = joy_command_rec.axes[1];
    joy_z_rec.data = joy_command_rec.axes[2];
    joy_yaw_rec.data = joy_command_rec.axes[3]; 
    ctrl_flag = 2;  
}

void dji_rollpitch_yawrate_z(const sensor_msgs::Joy::ConstPtr& msg){
    joy_command_rec = *msg;
    joy_x_rec.data = joy_command_rec.axes[0];
    joy_y_rec.data = joy_command_rec.axes[1];
    joy_z_rec.data = joy_command_rec.axes[2];
    joy_yaw_rec.data = joy_command_rec.axes[3]; 
    ctrl_flag = 3;  
}

/*Callback de las llamadas a servicios de DJI*/

bool task_request(
       dji_px4::DroneTaskControl::Request &req,
       dji_px4::DroneTaskControl::Response &resp){
       dji_task = req.task;
       ROS_INFO("Comando DroneTaskControl --%d recibido, enviando a UAL", dji_task);
       return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    std::string dji_ns = "dji";

    std::string set_generic_topic = dji_ns + "/flight_control_setpoint_generic";
    std::string set_enuposition_topic = dji_ns + "/flight_control_setpoint_ENUposition_yaw";
    std::string set_enuvelocity_topic = dji_ns + "/flight_control_setpoint_ENUvelocity_yawrate";
    std::string set_rollpitch_yaw_topic = dji_ns + "/flight_control_setpoint_rollpitch_yawrate_zposition";

    /*Creamos clientes para llamadas a servicios de UAL*/

    ros::ServiceClient takeoff_client = nh.serviceClient<uav_abstraction_layer::TakeOff>
            ("/ual/take_off");

    ros::ServiceClient land_client = nh.serviceClient<uav_abstraction_layer::Land>
            ("/ual/land");

    ros::ServiceClient sethome_client = nh.serviceClient<uav_abstraction_layer::SetHome>
            ("/ual/set_home");

    /*Servicios de DJI*/

    ros::ServiceServer server = nh.advertiseService("/dji_sdk/drone_task_control",&task_request);
    ros::ServiceClient dji_takeoff_client = nh.serviceClient<dji_px4::DroneTaskControl>
            ("/dji_sdk/drone_task_control");

    /*Suscriptores de MAVROS*/

    ros::Subscriber mavros_state = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix> /*Global Position*/
            ("mavros/global_position/global", 10, gps_cb);

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped> /*Local Position*/
            ("ual/pose", 10, pose_cb);


    /*Suscriptores para caso real*/
    ros::Subscriber dji_setpoint_generic_sub = nh.subscribe<sensor_msgs::Joy>
        ("/dji_sdk/flight_control_setpoint_generic", 10, dji_setpoint_generic);

    ros::Subscriber dji_enuposition_yaw_sub = nh.subscribe<sensor_msgs::Joy>
        ("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10, dji_enuposition_yaw);

    ros::Subscriber dji_enuvelocity_yawrate_sub = nh.subscribe<sensor_msgs::Joy>
        ("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10,  dji_enuvelocity_yawrate);

    ros::Subscriber dji_rollpitch_yawrate_zpos_sub = nh.subscribe<sensor_msgs::Joy>
        ("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10, dji_rollpitch_yawrate_z);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/ual/set_pose", 10);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/ual/set_velocity", 10);

    ros::Publisher local_attitude_pub = nh.advertise<uav_abstraction_layer::AttSetpoint>
            ("/ual/set_attitude", 10);

    ros::Publisher local_attitude_rate_pub = nh.advertise<uav_abstraction_layer::AttRateSetpoint>
            ("/ual/set_attitude_rate", 10);

    /*Publicadores que envian comandos*/
    ros::Publisher dji_enuposition_yaw_pub = nh.advertise<sensor_msgs::Joy>
            ("/dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

    ros::Publisher dji_enuvelocity_yawrate_pub = nh.advertise<sensor_msgs::Joy>
            ("/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);

    ros::Publisher dji_rollpitch_yawrate_zpos_pub = nh.advertise<sensor_msgs::Joy>
            ("/dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped twist;
    uav_abstraction_layer::AttSetpoint attitude;
    uav_abstraction_layer::AttRateSetpoint attitude_rate;
    uav_abstraction_layer::TakeOff::Request takeoff_req;
    uav_abstraction_layer::TakeOff::Response takeoff_resp;
    uav_abstraction_layer::Land::Request land_req;
    uav_abstraction_layer::Land::Response land_resp;
    uav_abstraction_layer::SetHome::Request gohome_req;
    uav_abstraction_layer::SetHome::Response gohome_resp;
    dji_px4::DroneTaskControl::Request dji_takeoff_req;
    dji_px4::DroneTaskControl::Response dji_takeoff_resp;


    int r = 0;
    int c = 0;

    while(ros::ok()){

        if(ctrl_flag == 0){
            ROS_INFO("Control generic...");

            /*LECTURA FLAG HORIZONTAL*/
            r = flag%64;
            c = flag/64;
            if(c == 0){
                attitude.orientation.x = current_pose.pose.orientation.x + joy_x_rec.data; /*roll*/
                attitude.orientation.y = current_pose.pose.orientation.y + joy_y_rec.data; /*pitch*/
            }
            else if(c == 1){
                twist.twist.linear.x = joy_x_rec.data; /*velocidad en x*/
                twist.twist.linear.y = joy_y_rec.data; /*velocidad en y*/
            }
            else if(c == 2){
                pose.pose.position.x = current_pose.pose.position.x + joy_x_rec.data; /*X offset*/
                pose.pose.position.y = current_pose.pose.position.y; + joy_y_rec.data; /*Y offset*/
            }
            else {
                pose.pose.position.x = current_pose.pose.position.x + joy_x_rec.data; /*X offset*/
                pose.pose.position.y = current_pose.pose.position.y; + joy_y_rec.data; /*Y offset*/
            }

            /*LECTURA DE FLAG VERTICAL*/
            c = r/16;
            r = r%16;

            if(c == 0){
                twist.twist.linear.z = joy_z_rec.data; /*velocidad en z*/
            }
            else if(c == 1){
                pose.pose.position.z = joy_z_rec.data; /*height*/
            }
            else{
                attitude.thrust = joy_z_rec.data/100; /*thrust (pasando de 0-100 a 0-1)*/
            }

            /*LECTURA DE FLAG YAW*/
            c = r/8;
            r = r%8;

            if(c == 0){
                pose.pose.orientation.z = joy_yaw_rec.data; /*yaw angle en modo LOCAL_POSE DE UAL*/
            }
            else{
                twist.twist.angular.z = joy_yaw_rec.data; /*yawrate*/
                /*attitude_rate.body_rate.z = joy_z_rec.data;  /*yaw rate en modo ATTITUDE de UAL*/
            }

            /*LECTURA FLAG DE COORDENADAS: rosparam set /ual/pose_frame_id*/
            c = r/2;
            r = r%2;
            
            if(c == 0){ /*frame GROUND ENU*
                /*pose.header.frame_id = '';*/
                /*attitude.header.frame_id = ;*/
                /*twist.header.frame_id = ;*/
            }
            else{ /*frame BODY FLU*/
               /*pose.header.frame_id = '';*/
                /*attitude.header.frame_id = ;*/
                /*twist.header.frame_id = ;*/
            }

            /*LECTURA FLAG ACTIVE BREAK*/

            if(r == 0){} /*ACTIVE BREAK DESACTIVADO*/
            else{} /*ACTIVE BREAK ACTIVADO (por defecto en UAL*/

        }
        else if(ctrl_flag == 1){ /*para /dji_sdk/flight_control_setpoint_ENUposition_yaw*/
            ROS_INFO("Controlando en posicion...");
            pose.pose.position.x = current_pose.pose.position.x + joy_x_rec.data; /*X offset*/
            pose.pose.position.y = current_pose.pose.position.y + joy_y_rec.data; /*Y offset*/
            pose.pose.position.z = joy_z_rec.data; /*height*/
            pose.pose.orientation.z = current_pose.pose.orientation.z + joy_yaw_rec.data; /*yaw angle en modo LOCAL_POSE DE UAL*/
            local_pos_pub.publish(pose);          
        }

        else if(ctrl_flag == 2) { /*para /dji_sdk/flight_control_setpoint_ENUvelocity_yawrate*/
            ROS_INFO("Controlando en velocidad...");
            twist.twist.linear.x = joy_x_rec.data; /*velocidad en x*/
            twist.twist.linear.y = joy_y_rec.data; /*velocidad en y*/
            twist.twist.linear.z = joy_z_rec.data; /*velocidad en z*/
            twist.twist.angular.z = joy_yaw_rec.data; /*yawrate*/
            local_vel_pub.publish(twist);
        }

        else if (ctrl_flag == 3) { /*para /dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition*/
            ROS_INFO("Controlando en angulo...");
            attitude.orientation.x = current_pose.pose.orientation.x + joy_x_rec.data; /*roll*/
            attitude.orientation.y = current_pose.pose.orientation.y + joy_y_rec.data; /*pitch*/
            pose.pose.position.z = current_pose.pose.position.z + joy_z_rec.data;    /*height*/
            attitude_rate.body_rate.z = joy_yaw_rec.data;  /*yaw rate en modo ATTITUDE de UAL*/
            attitude.thrust = 0.5; /*necesito especificar este valor en UAL para controlar en angulo*/
            attitude_rate.thrust = attitude.thrust; 
            local_attitude_pub.publish(attitude);
            /*local_attitude_rate_pub.publish(attitude_rate);*/
            }

        /*Aquí va el remapeo de servicios*/
        else if (ctrl_flag == 4){
            if(dji_task == 4){
                takeoff_req.height = takeoff_height;
                takeoff_req.blocking = blocking;
                if(takeoff_client.call(takeoff_req, takeoff_resp)) ROS_INFO("Takeoff Success");
                dji_task = 0;
            }

            else if(dji_task == 6){
                land_req.blocking = blocking; 
                if(land_client.call(land_req, land_resp)) ROS_INFO("Land Success");
                dji_task = 0;
            }
            else if(dji_task == 1){
                gohome_req.set_z = 1; 
                if(sethome_client.call(gohome_req, gohome_resp)) ROS_INFO("Set Home Success");
                dji_task = 0;
            }
        }
        ros::spinOnce();
        rate.sleep();  
    }

    return 0;
}
