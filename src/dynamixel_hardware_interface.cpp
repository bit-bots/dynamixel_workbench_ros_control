#include <dynamixel_workbench_ros_control/dynamixel_hardware_interface.h>
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))

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
  switchDynamixelControlMode();

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

  // alloc memory for imu values
  orientation_ = (double*) malloc(4 * sizeof(double));
  std::fill(orientation_, orientation_+4, 0);
  orientation_covariance_ = (double*) malloc(9 * sizeof(double));
  std::fill(orientation_covariance_, orientation_covariance_+9, 0);
  angular_velocity_ = (double*) malloc(3 * sizeof(double));
  std::fill(angular_velocity_, angular_velocity_+3, 0);
  angular_velocity_covariance_ = (double*) malloc(9 * sizeof(double));
  std::fill(angular_velocity_covariance_, angular_velocity_covariance_+9, 0);
  linear_acceleration_ = (double*) malloc(3 * sizeof(double));
  std::fill(linear_acceleration_, linear_acceleration_+3, 0);
  linear_acceleration_covariance_ = (double*) malloc(9 * sizeof(double));
  std::fill(linear_acceleration_covariance_, linear_acceleration_covariance_+9, 0);

  std::string imu_name;
  std::string imu_frame;
  nh.getParam("IMU/name", imu_name);
  nh.getParam("IMU/frame", imu_frame);
  nh.getParam("IMU/read", read_imu_);
  hardware_interface::ImuSensorHandle imu_handle(imu_name, imu_frame, orientation_, orientation_covariance_, angular_velocity_, angular_velocity_covariance_, linear_acceleration_, linear_acceleration_covariance_);
  imu_interface_.registerHandle(imu_handle);
  registerInterface(&imu_interface_);

  ROS_INFO("Hardware interface init finished.");
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
  driver_->addSyncWrite("Operating_Mode");
  driver_->addSyncRead("Present_Current");
  driver_->addSyncRead("Present_Velocity");
  driver_->addSyncRead("Present_Position");

  return success;
}

void DynamixelHardwareInterface::setTorque(bool enabled)
{
  std::vector<int32_t> torque(joint_names_.size(), enabled);
  int32_t* t = &torque[0];
  driver_->syncWrite("Torque_Enable", t);
  current_torque_ = enabled;
}

void DynamixelHardwareInterface::setTorque(std_msgs::BoolConstPtr enabled)
{
  // we save the goal torque value. It will be set during write process
  goal_torque_ = enabled->data;
}

void DynamixelHardwareInterface::read()
{
  if (read_position_ && read_velocity_ && read_effort_ ){
    if(syncReadAll()){
      for (size_t num = 0; num < joint_names_.size(); num++)
        current_position_[num] += joint_mounting_offsets_[num] + joint_offsets_[num];
    } else
      ROS_ERROR_THROTTLE(1.0, "Couldn't read all current joint values!");
  }else {
    if (read_position_) {
      if (syncReadPositions()) {
        for (size_t num = 0; num < joint_names_.size(); num++)
          current_position_[num] += joint_mounting_offsets_[num] + joint_offsets_[num];
      } else
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint position!");
    }

    if (read_velocity_) {
      if (!syncReadVelocities()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint velocity!");
      }
    }

    if (read_effort_) {
      if (!syncReadEfforts()) {
        ROS_ERROR_THROTTLE(1.0, "Couldn't read current joint effort!");
      }
    }
  }

  if (first_cycle_)
  {
    goal_position_ = current_position_;
    first_cycle_ = false;
  }

  if(read_imu_){
      if(!readImu()){
          ROS_ERROR_THROTTLE(1.0, "Couldn't read IMU");
      }
  }
}

void DynamixelHardwareInterface::write()
{
  //check if we have to switch the torque
  if(current_torque_ != goal_torque_){
    setTorque(goal_torque_);
  }

  if (control_mode_ == PositionControl)
  {
      syncWritePosition();
  } else if (control_mode_ == VelocityControl)
  {
      syncWriteVelocity();
  } else if (control_mode_ == EffortControl)
  {
      syncWriteCurrent();
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

  std::vector<int32_t> operating_mode(joint_names_.size(), value);
  int32_t* o = &operating_mode[0];
  driver_->syncWrite("Operating_Mode", o);

  ros::Duration(0.5).sleep();
  //reenable torque
  setTorque(true);
}

bool DynamixelHardwareInterface::syncReadPositions(){
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Position", data);
  for(int i = 0; i < joint_count_; i++){
    current_position_[i] = driver_->convertValue2Radian(joint_ids_[i], data[i]);
  }

  free(data);
  return success;
}

bool DynamixelHardwareInterface::syncReadVelocities(){
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Velocity", data);
  for(int i = 0; i < joint_count_; i++){
    current_velocity_[i] = driver_->convertValue2Velocity(joint_ids_[i], data[i]);
  }
  free(data);

  return success;
}

bool DynamixelHardwareInterface::syncReadEfforts() {
  bool success;
  int32_t *data = (int32_t *) malloc(joint_count_ * sizeof(int32_t));
  success = driver_->syncRead("Present_Current", data);
  for (int i = 0; i < joint_count_; i++) {
    current_effort_[i] = driver_->convertValue2Torque(joint_ids_[i], data[i]);
  }
  free(data);

  return success;
}

bool DynamixelHardwareInterface::syncReadAll() {
  bool success;
  std::vector<uint8_t> data;
  if(driver_->syncReadMultipleRegisters(126, 10, &data)) {
    uint32_t eff;
    uint32_t vel;
    uint32_t pos;
    for (int i = 0; i < joint_count_; i++) {
      eff = DXL_MAKEWORD(data[i * 10], data[i * 10 + 1]);
      vel = DXL_MAKEDWORD(DXL_MAKEWORD(data[i * 10 + 2], data[i * 10 + 3]),
                          DXL_MAKEWORD(data[i * 10 + 4], data[i * 10 + 5]));
      pos = DXL_MAKEDWORD(DXL_MAKEWORD(data[i * 10 + 6], data[i * 10 + 7]),
                          DXL_MAKEWORD(data[i * 10 + 8], data[i * 10 + 9]));
      current_effort_[i] = driver_->convertValue2Torque(joint_ids_[i], eff);
      current_velocity_[i] = driver_->convertValue2Velocity(joint_ids_[i], vel);
      current_position_[i] = driver_->convertValue2Radian(joint_ids_[i], pos);
    }
    return true;
  }else{
    return false;
  }
}

bool DynamixelHardwareInterface::readImu(){
    bool success;
    uint8_t *data = (uint8_t *) malloc(110 * sizeof(uint8_t));

    if(driver_->readMultipleRegisters(241, 36, 16, data)){
      //todo we have to check if we jumped one sequence number
        uint32_t highest_seq_number = 0;
        uint32_t new_value_index=0;
        uint32_t current_seq_number= 0;
        // imu gives us 2 values at the same time, lets see which one is the newest
        for(int i =0; i < 2; i++){
            //the sequence number are the bytes 12 to 15 for each of the two 16 Bytes
            current_seq_number = DXL_MAKEDWORD(DXL_MAKEWORD(data[16*i+12], data[16*i+13]),
                                             DXL_MAKEWORD(data[16*i+14], data[16*i+15]));
          if(current_seq_number>highest_seq_number){
              highest_seq_number=current_seq_number;
              new_value_index=i;
            }
        }
      // linear acceleration are two signed bytes with 256 LSB per g
      linear_acceleration_[0] = ((short) DXL_MAKEWORD(data[16*new_value_index], data[16*new_value_index+1])) / 256.0 ;
      linear_acceleration_[1] = ((short) DXL_MAKEWORD(data[16*new_value_index+2], data[16*new_value_index+3])) / 256.0 ;
      linear_acceleration_[2] = ((short)DXL_MAKEWORD(data[16*new_value_index+4], data[16*new_value_index+5])) / 256.0 ;
      // angular velocity are two signed bytes with 14.375 per deg/s
      angular_velocity_[0] = ((short)DXL_MAKEWORD(data[16*new_value_index+6], data[16*new_value_index+7])) / 14.375;
      angular_velocity_[1] = ((short)DXL_MAKEWORD(data[16*new_value_index+8], data[16*new_value_index+9])) / 14.375;
      angular_velocity_[2] = ((short)DXL_MAKEWORD(data[16*new_value_index+10], data[16*new_value_index+11])) / 14.375;

      return true;
    }else {
      return false;
    }
}

bool DynamixelHardwareInterface::syncWritePosition(){
  int* goal_position = (int*)malloc(joint_names_.size() * sizeof(int));
  float radian;
  for (size_t num = 0; num < joint_names_.size(); num++) {
    radian = goal_position_[num] - joint_mounting_offsets_[num] - joint_offsets_[num];
    goal_position[num] = driver_->convertRadian2Value(joint_ids_[num], radian);
  }
  driver_->syncWrite("Goal_Position", goal_position);
  free(goal_position);
}

bool DynamixelHardwareInterface::syncWriteVelocity() {
  int* goal_velocity = (int*)malloc(joint_names_.size() * sizeof(int));
  for (size_t num = 0; num < joint_names_.size(); num++) {
    goal_velocity[num] = driver_->convertRadian2Value(joint_ids_[num], goal_velocity_[num]);
  }
  driver_->syncWrite("Goal_Velocity", goal_velocity);
  free(goal_velocity);
}

bool DynamixelHardwareInterface::syncWriteCurrent() {
  int* goal_current = (int*)malloc(joint_names_.size() * sizeof(int));
  for (size_t num = 0; num < joint_names_.size(); num++) {
    goal_current[num] = driver_->convertRadian2Value(joint_ids_[num], goal_effort_[num]);
  }
  driver_->syncWrite("Goal_Current", goal_current);
  free(goal_current);
}


}
