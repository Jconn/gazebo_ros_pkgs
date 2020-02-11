//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_plugins/gazebo_ros_magnetic_sensor.hpp>

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace gazebo_plugins {

class GazeboRosMagneticSensorPrivate
{
public:
  /// Node for ros communication
  gazebo_ros::Node::SharedPtr ros_node_;
  /// Publish for gps message
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_;
  /// GPS message modified each update
  sensor_msgs::msg::MagneticField::SharedPtr msg_;
  /// GPS sensor this plugin is attached to
  gazebo::sensors::MagnetometerSensorPtr sensor_;
  /// Event triggered when sensor updates
  gazebo::event::ConnectionPtr sensor_update_event_;

  /// Publish latest gps data to ROS
  void OnUpdate();
};

GazeboRosMagneticSensor::GazeboRosMagneticSensor()
: impl_(std::make_unique<GazeboRosMagneticSensorPrivate>())
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMagneticSensor::~GazeboRosMagneticSensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosMagneticSensor::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{

  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->sensor_ = std::dynamic_pointer_cast<gazebo::sensors::MagnetometerSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not a magnetometer sensor. Exiting.");
    return;
  }

  impl_->pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::MagneticField>(
    "~/out", rclcpp::SensorDataQoS());

  // Create message to be reused
  auto msg = std::make_shared<sensor_msgs::msg::MagneticField>();

  // Get frame for message
  msg->header.frame_id = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  // Fill covariances
  using SNT = gazebo::sensors::SensorNoiseType;

  msg->magnetic_field_covariance[0] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::MAGNETOMETER_X_NOISE_TESLA));
  msg->magnetic_field_covariance[4] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::MAGNETOMETER_Y_NOISE_TESLA));
  msg->magnetic_field_covariance[8] =
    gazebo_ros::NoiseVariance(impl_->sensor_->Noise(SNT::MAGNETOMETER_Z_NOISE_TESLA));

  impl_->msg_ = msg;

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosMagneticSensorPrivate::OnUpdate, impl_.get()));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMagneticSensorPrivate::OnUpdate()
{

  msg_->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());
  ignition::math::Vector3d cur_field = sensor_->MagneticField();
  msg_->magnetic_field.x = cur_field.X(); 
  msg_->magnetic_field.y = cur_field.Y(); 
  msg_->magnetic_field.z = cur_field.Z();
  pub_->publish(*msg_);                
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMagneticSensor)

} // namespace gazebo
