
#include <ros/ros.h>
#include <ros/console.h>
#include <chrono>
#include <ctime>
#include <std_msgs/String.h>
#include <signal.h>

#define TIME_STR_LENGTH 17

ros::Publisher register_pub;
ros::Publisher remove_pub;

std_msgs::String msg;

void interruptCallback(int sig) {
    remove_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_service_lab_turtlebot3_register");
    ros::NodeHandle nh;
    ros::NodeHandle privateNh("~");

    register_pub = nh.advertise<std_msgs::String>("/turtlebot_service/robot_manager/register", 10);
    remove_pub = nh.advertise<std_msgs::String>("/turtlebot_service/robot_manager/remove", 10);

    ros::Rate loop_rate(1);

    // Setting the message
    if (!privateNh.getParam("robot_id", msg.data)) {
        ROS_WARN_STREAM("Robot id is not set");
    }

    while (ros::ok())
    {
        ROS_INFO("Robot register: %s", msg.data.c_str());
        register_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    signal(SIGINT, interruptCallback);
    ros::shutdown();

    return 0;
}


