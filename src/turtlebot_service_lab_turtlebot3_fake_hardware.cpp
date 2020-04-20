/*******************************************************************************
* Disclaimer:
* Modified version of turtlebot3/turtlebot3_diagnostic node
*******************************************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/VersionInfo.h>

ros::Publisher imu_pub;
ros::Publisher scan_pub;
ros::Publisher sensor_state_pub;
ros::Publisher firmware_version_pub;

void msgPub() {
    sensor_msgs::Imu imuMsg;
    sensor_msgs::LaserScan scanMsg;
    imu_pub.publish(imuMsg);
    scan_pub.publish(scanMsg);

    turtlebot3_msgs::SensorState ssMsg;
    float battery; int button; bool torque;
    ros::NodeHandle privateNh("~");
    if (!privateNh.getParam("battery", battery)) battery = 1.0;
    if (!privateNh.getParam("button", button))   button = 0;
    if (!privateNh.getParam("torque", torque))   torque = false;
    ssMsg.battery = battery;
    ssMsg.button  = button;
    ssMsg.torque  = torque;
    sensor_state_pub.publish(ssMsg);

    turtlebot3_msgs::VersionInfo versionMsg;
    versionMsg.firmware = "1.2.3";
    versionMsg.hardware = "2020.03.16";
    versionMsg.software = "1.2.0";
    firmware_version_pub.publish(versionMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlebot_service_lab_turtlebot3_fake_hardware");
    ros::NodeHandle nh;

    imu_pub              = nh.advertise<sensor_msgs::Imu>("imu", 10);
    scan_pub             = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
    sensor_state_pub     = nh.advertise<turtlebot3_msgs::SensorState>("sensor_state", 10);
    firmware_version_pub = nh.advertise<turtlebot3_msgs::VersionInfo>("firmware_version", 10);

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        msgPub();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
