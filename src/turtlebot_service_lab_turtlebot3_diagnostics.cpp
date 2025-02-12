/*******************************************************************************
* Disclaimer:
* Modified version of turtlebot3/turtlebot3_diagnostic node
*******************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <turtlebot3_msgs/SensorState.h>
#include <turtlebot3_msgs/VersionInfo.h>
#include <string>

ros::Publisher tb3_version_info_pub;
ros::Publisher tb3_diagnostics_pub;

diagnostic_msgs::DiagnosticStatus imu_state;
diagnostic_msgs::DiagnosticStatus motor_state;
diagnostic_msgs::DiagnosticStatus LDS_state;
diagnostic_msgs::DiagnosticStatus battery_state;
diagnostic_msgs::DiagnosticStatus button_state;

void split(std::string data, std::string separator, std::string* temp)
{
	int cnt = 0;
  std::string copy = data;
  
	while(true)
	{
		std::size_t index = copy.find(separator);

    if (index != std::string::npos)
    {
      temp[cnt] = copy.substr(0, index);

      copy = copy.substr(index+1, copy.length());
    }
    else
    {
      temp[cnt] = copy.substr(0, copy.length());
      break;
    }
    
		++cnt;
	}
}

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name  = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

void setIMUDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "MPU9250");
}

void setMotorDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&motor_state, level, "Actuator", message, "DYNAMIXEL X");
}

void setBatteryDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&battery_state, level, "Power System", message, "Battery");
}

void setLDSDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&LDS_state, level, "Lidar Sensor", message, "HLS-LFCD-LDS");
}

void setButtonDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&button_state, level, "Analog Button", message, "OpenCR Button");
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void sensorStateMsgCallback(const turtlebot3_msgs::SensorState::ConstPtr &msg)
{
  if (msg->battery > 11.0)
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  else
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Charge!!! Charge!!!");

  if (msg->button == turtlebot3_msgs::SensorState::BUTTON0)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 0 IS PUSHED");
  else if (msg->button == turtlebot3_msgs::SensorState::BUTTON1)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 1 IS PUSHED");
  else
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Pushed Nothing");

  if (msg->torque == true)
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Torque ON");
  else
    setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Torque OFF");
}

void firmwareVersionMsgCallback(const turtlebot3_msgs::VersionInfo msg)
{
  tb3_version_info_pub.publish(msg);
}

bool isEmpty(diagnostic_msgs::DiagnosticStatus status) {
  return status.name.empty() || status.message.empty();
}

void msgPub()
{
  diagnostic_msgs::DiagnosticArray tb3_diagnostics;

  tb3_diagnostics.header.stamp = ros::Time::now();

  tb3_diagnostics.status.clear();
  if ( !isEmpty(imu_state) )     tb3_diagnostics.status.push_back(imu_state);
  if ( !isEmpty(motor_state) )   tb3_diagnostics.status.push_back(motor_state);
  if ( !isEmpty(LDS_state) )     tb3_diagnostics.status.push_back(LDS_state);
  if ( !isEmpty(battery_state) ) tb3_diagnostics.status.push_back(battery_state);
  if ( !isEmpty(button_state) )  tb3_diagnostics.status.push_back(button_state);

  tb3_diagnostics_pub.publish(tb3_diagnostics);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_service_lab_turtlebot3_diagnostics");
  ros::NodeHandle nh;

  tb3_diagnostics_pub  = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  tb3_version_info_pub = nh.advertise<turtlebot3_msgs::VersionInfo>("version_info", 10);

  ros::Subscriber imu         = nh.subscribe("imu", 10, imuMsgCallback);
  ros::Subscriber lds         = nh.subscribe("scan", 10, LDSMsgCallback);
  ros::Subscriber tb3_sensor  = nh.subscribe("sensor_state", 10, sensorStateMsgCallback);
  ros::Subscriber version     = nh.subscribe("firmware_version", 10, firmwareVersionMsgCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
