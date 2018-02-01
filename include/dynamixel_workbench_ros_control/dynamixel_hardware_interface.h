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

  boost::shared_ptr<DynamixelDriver> driver_;

  hardware_interface::JointStateInterface jnt_state_interface_;

  hardware_interface::PositionJointInterface jnt_pos_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  hardware_interface::EffortJointInterface jnt_eff_interface_;

  hardware_interface::ImuSensorInterface imu_interface_;

  ControlMode control_mode_;

  int joint_count_;

  std::vector<std::string> joint_names_;
  std::vector<uint8_t> joint_ids_;
  std::vector<double> joint_mounting_offsets_;
  std::vector<double> joint_offsets_;

  std::vector<double> goal_position_;
  std::vector<double> goal_effort_;
  std::vector<double> goal_velocity_;

  bool read_position_;
  bool read_velocity_;
  bool read_effort_;
  std::vector<double> current_position_;
  std::vector<double> current_velocity_;
  std::vector<double> current_effort_;

  bool read_imu_;
    uint32_t last_seq_number_;
   double* orientation_; //quaternion (x,y,z,w)
   double* orientation_covariance_;
  double* angular_velocity_;
   double* angular_velocity_covariance_;
    double* linear_acceleration_;
   double* linear_acceleration_covariance_;

  // subscriber
  ros::Subscriber set_torque_sub_;
};
}

#endif
