#ifndef DYNAMIXEL_HARWARE_INTERFACE_H
#define DYNAMIXEL_HARWARE_INTERFACE_H

#include <ros/ros.h>

#include <std_msgs/Bool.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

#include <dynamixel_workbench/dynamixel_driver.h>

namespace dynamixel_workbench_ros_control
{
template<typename T>
std::string vecToString(const std::vector<T>& vec)
{
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() -1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

struct State
{
  State() : position(0), velocity(0), effort(0) {}
  double position;
  double velocity;
  double effort;
};

struct Joint
{
  std::string name;
  State current;
  State goal;
};

enum ControlMode {
  PositionControl,
  VelocityControl,
  EffortControl
};

class DynamixelHardwareInterface : public hardware_interface::RobotHW
{
public:
  DynamixelHardwareInterface();

  bool init(ros::NodeHandle& nh);
  void read();
  void write();

private:
  bool loadDynamixels(ros::NodeHandle& nh);
  bool stringToControlMode(std::string control_mode_str, ControlMode &control_mode);
  bool switchDynamixelControlMode();

  bool goal_torque_;
  bool current_torque_;
  void setTorque(bool enabled);
  void setTorque(std_msgs::BoolConstPtr enabled);

  bool syncReadPositions();
  bool syncReadVelocities();
  bool syncReadEfforts();
  bool syncReadAll();
  bool readImu();

  bool syncWritePosition();
  bool syncWriteVelocity();
  bool syncWriteCurrent();

  bool first_cycle_;

  boost::shared_ptr<DynamixelDriver> _driver;

  hardware_interface::JointStateInterface _jnt_state_interface;

  hardware_interface::PositionJointInterface _jnt_pos_interface;
  hardware_interface::VelocityJointInterface _jnt_vel_interface;
  hardware_interface::EffortJointInterface _jnt_eff_interface;

  hardware_interface::ImuSensorInterface _imu_interface;

  ControlMode _control_mode;

  int _joint_count;

  std::vector<std::string> _joint_names;
  std::vector<uint8_t> _joint_ids;
  std::vector<double> _joint_mounting_offsets;
  std::vector<double> _joint_offsets;

  std::vector<double> _goal_position;
  std::vector<double> _goal_effort;
  std::vector<double> _goal_velocity;

  bool _read_position;
  bool _read_velocity;
  bool _read_effort;
  std::vector<double> _current_position;
  std::vector<double> _current_velocity;
  std::vector<double> _current_effort;

  bool _read_imu;
    uint32_t _last_seq_number;
   double* _orientation; //quaternion (x,y,z,w)
   double* _orientation_covariance;
  double* _angular_velocity;
   double* _angular_velocity_covariance;
    double* _linear_acceleration;
   double* _linear_acceleration_covariance;

  // subscriber
  ros::Subscriber _set_torque_sub;
};
}

#endif
