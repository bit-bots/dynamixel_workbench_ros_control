#include <dynamixel_workbench_ros_control/dynamixel_hardware_interface.h>

namespace dynamixel_workbench_ros_control
{

DynamixelHardwareInterface::DynamixelHardwareInterface()
  : first_cycle_(true), read_position_(true), read_velocity_(false), read_effort_(true),driver_(new DynamixelDriver())
{}

bool DynamixelHardwareInterface::init(ros::NodeHandle& nh)
{

  // Init subscriber
  set_torque_sub_ = nh.subscribe<std_msgs::BoolConstPtr>("set_torque", 1, &DynamixelHardwareInterface::setTorque, this);

  // Load dynamixel config from parameter server
  if (!loadDynamixels(nh))
  {
    ROS_ERROR_STREAM("Failed to ping all motors.");
    return false;
  }

  // Switch dynamixels to correct control mode (position, velocity, effort)
  //switchDynamixelControlMode();

  joint_count_ = joint_names_.size();
  current_position_.resize(joint_count_, 0);
  current_velocity_.resize(joint_count_, 0);
  current_effort_.resize(joint_count_, 0);
  goal_position_.resize(joint_count_, 0);
  goal_velocity_.resize(joint_count_, 0);
  goal_effort_.resize(joint_count_, 0);
  // register interfaces
  for (unsigned int i = 0; i < joint_names_.size(); i++)
  {
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &current_position_[i], &current_velocity_[i], &current_effort_[i]);
    jnt_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(state_handle, &goal_position_[i]);
    jnt_pos_interface_.registerHandle(pos_handle);

    hardware_interface::JointHandle vel_handle(state_handle, &goal_velocity_[i]);
    jnt_vel_interface_.registerHandle(vel_handle);

    hardware_interface::JointHandle eff_handle(state_handle, &goal_effort_[i]);
    jnt_eff_interface_.registerHandle(eff_handle);

  }
  registerInterface(&jnt_state_interface_);
  if (control_mode_ == PositionControl)
  {
    registerInterface(&jnt_pos_interface_);
  } else if (control_mode_ == VelocityControl)
  {
    registerInterface(&jnt_vel_interface_);
  } else if (control_mode_ == EffortControl)
  {
    registerInterface(&jnt_eff_interface_);
  }
  setTorque(nh.param("auto_torque", false));
  ROS_WARN("init finished");
  return true;
}

bool DynamixelHardwareInterface::loadDynamixels(ros::NodeHandle& nh)
{
  bool success = true;

  ROS_INFO_STREAM("Loading parameters from namespace " << nh.getNamespace() + "/dynamixels");

  // get control mode
  std::string control_mode;
  nh.getParam("dynamixels/control_mode", control_mode);
  if (!stringToControlMode(control_mode, control_mode_)) {
    ROS_ERROR_STREAM("Unknown control mode'" << control_mode << "'.");
    return false;
  }

  // get values to read
  nh.param("dynamixels/read_values/read_position", read_position_, true);
  nh.param("dynamixels/read_values/read_velocity", read_velocity_, false);
  nh.param("dynamixels/read_values/read_effort", read_effort_, false);

  // get port info
  std::string port_name;
  nh.getParam("dynamixels/port_info/port_name", port_name);
  int baudrate;
  nh.getParam("dynamixels/port_info/baudrate", baudrate);
  float protocol_version;
  nh.getParam("dynamixels/port_info/protocol_version", protocol_version);
  driver_->init(port_name.c_str(), uint32_t(baudrate));

  XmlRpc::XmlRpcValue dxls;
  nh.getParam("dynamixels/device_info", dxls);
  ROS_ASSERT(dxls.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  int i = 0;
  for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = dxls.begin(); it != dxls.end(); ++it)
  {
    std::string dxl_name = (std::string)(it->first);
    joint_names_.push_back(dxl_name);
    ros::NodeHandle dxl_nh(nh, "dynamixels/device_info/" + dxl_name);

    joint_mounting_offsets_.push_back(dxl_nh.param("mounting_offset", 0.0));
    joint_offsets_.push_back(dxl_nh.param("offset", 0.0));

    int motor_id;
    dxl_nh.getParam("id", motor_id);

    int model_number;
    dxl_nh.getParam("model_number", model_number);
    uint16_t model_number_16 = uint16_t(model_number);
    uint16_t* model_number_16p = &model_number_16;

    //ping it to very that it's there and to add it to the driver
    if(!driver_->ping(uint8_t(motor_id), model_number_16p)){
      ROS_ERROR("Was not able to ping motor with id %d", motor_id);
      success = false;
    }
    joint_ids_.push_back(uint8_t(motor_id));
    i++;
  }
  if(!success){
    return false;
  }

  driver_->setPacketHandler(protocol_version);
  driver_->addSyncWrite("Torque_Enable");
  driver_->addSyncWrite("Goal_Position");
  driver_->addSyncWrite("Goal_Velocity");
  driver_->addSyncWrite("Goal_Current");
  driver_->addSyncRead("Present_Current");
  driver_->addSyncRead("Present_Velocity");
  driver_->addSyncRead("Present_Position");





  return success;
}

void DynamixelHardwareInterface::setTorque(bool enabled)
{
  std::vector<uint8_t> torque(joint_names_.size(), enabled);
  //syncWriteTorque(torque);
}

void DynamixelHardwareInterface::setTorque(std_msgs::BoolConstPtr enabled)
{
  setTorque(enabled->data);
}

void DynamixelHardwareInterface::read()
{
  if (read_position_)
  {
    if (syncReadPositions())
    {
      for (size_t num = 0; num < joint_names_.size(); num++)
        current_position_[num] += joint_mounting_offsets_[num] + joint_offsets_[num];
    }
    else
      ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint position!");
  }

  if (read_velocity_)
  {
    if (!syncReadVelocities(current_velocity_))
    {
      ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint velocity!");
    }
  }

  if (read_effort_)
  {
    if (!syncReadEfforts(current_effort_))
    {
      ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint effort!");
    }
  }

  if (first_cycle_)
  {
    goal_position_ = current_position_;
    first_cycle_ = false;
  }
}

void DynamixelHardwareInterface::write()
{
  if (control_mode_ == PositionControl)
  {

    std::vector<double> goal_position(joint_names_.size());
    for (size_t num = 0; num < joint_names_.size(); num++)
      goal_position[num] = goal_position_[num] - joint_mounting_offsets_[num] - joint_offsets_[num];
    //driver_->syncWritePosition(goal_position);
  } else if (control_mode_ == VelocityControl)
  {
    //driver_->syncWriteVelocity(goal_velocity_);
  } else if (control_mode_ == EffortControl)
  {
    //driver_->syncWriteCurrent(goal_effort_);
  }
}

bool DynamixelHardwareInterface::stringToControlMode(std::string control_mode_str, ControlMode& control_mode)
{
  if (control_mode_str == "position")
  {
    control_mode = PositionControl;
    return true;
  } else if (control_mode_str == "velocity")
  {
    control_mode = VelocityControl;
    return true;
  } else if (control_mode_str == "effort")
  {
    control_mode = EffortControl;
    return true;
  } else {
    return false;
  }
}

bool DynamixelHardwareInterface::switchDynamixelControlMode()
{
  // Torque on dynamixels has to be disabled to change operating mode
  setTorque(false);
  ros::Duration(0.5).sleep();

  int32_t value = 3;
  if (control_mode_ == PositionControl)
  {
    value = 3;;
  } else if (control_mode_ == VelocityControl)
  {
    value = 1;
  } else if (control_mode_ == EffortControl)
  {
    value = 0;
  }

  std::vector<uint8_t> operating_mode(joint_names_.size(), value);

  //driver_->syncWriteOperating(operating_mode);

  ros::Duration(0.5).sleep();
  //reenable torque
  setTorque(true);

}

bool DynamixelHardwareInterface::syncWriteTorqueEnable(bool torque) {
  int32_t torque32 = int32_t(torque);
  //return driver_->syncWrite("Torque_Enable", &torque32);
  return true;
}

bool DynamixelHardwareInterface::syncReadPositions(){
//ROS_WARN("reading positions");
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Position", data);
  for(int i = 0; i < joint_count_; i++){
    current_position_[i] = driver_->convertValue2Radian(joint_ids_[i], data[i]);
  }

  free(data);
  return success;
}

bool DynamixelHardwareInterface::syncReadVelocities(std::vector<double> velocities){
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Velocity", data);
  for(int i = 0; i < joint_count_; i++){
    current_velocity_[i] = driver_->convertValue2Velocity(joint_ids_[i], data[i]);
  }
  free(data);

  return success;
}

bool DynamixelHardwareInterface::syncReadEfforts(std::vector<double> efforts) {
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Current", data);
  for (int i = 0; i < joint_count_; i++) {
    current_effort_[i] = driver_->convertValue2Torque(joint_ids_[i], data[i]);
  }
  free(data);

  return success;
}

}
