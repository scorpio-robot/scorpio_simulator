// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SCORPIO_SIMULATOR__GLOBAL_ODOMETRY_PUBLISHER_HPP_
#define SCORPIO_SIMULATOR__GLOBAL_ODOMETRY_PUBLISHER_HPP_

#include <memory>
#include <random>
#include <string>

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/math/Pose3.hh"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace scorpio_simulator
{

/// \brief Private data class for GlobalOdometryPublisher
class GlobalOdometryPublisherPrivate
{
public:
  /// \brief ROS node for communication
  rclcpp::Node::SharedPtr ros_node_;

  /// \brief ROS publisher for odometry messages
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  /// \brief Model entity
  ignition::gazebo::Entity model_entity_;

  /// \brief Link entity (body to track)
  ignition::gazebo::Entity link_entity_;

  /// \brief Reference link entity (for relative pose)
  ignition::gazebo::Entity reference_link_entity_;

  /// \brief Gazebo child frame (link to track)
  std::string gazebo_child_frame_;

  /// \brief Gazebo parent frame (reference frame)
  std::string gazebo_frame_;

  /// \brief Topic name for publishing
  std::string topic_name_;

  /// \brief ROS odometry frame_id
  std::string ros_frame_id_;

  /// \brief ROS odometry child_frame_id
  std::string ros_child_frame_id_;

  /// \brief Robot namespace
  std::string robot_namespace_;

  /// \brief Compute twist in local frame
  bool local_twist_{false};

  /// \brief Constant offset pose
  ignition::math::Pose3d offset_;

  /// \brief Update rate (Hz), 0 means as fast as possible
  double update_rate_{0.0};

  /// \brief Gaussian noise standard deviation
  double gaussian_noise_{0.0};

  /// \brief Last update time
  std::chrono::steady_clock::duration last_update_time_{0};

  /// \brief Last linear velocity
  ignition::math::Vector3d last_vpos_;

  /// \brief Last angular velocity
  ignition::math::Vector3d last_veul_;

  /// \brief Random number generator for Gaussian noise
  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;

  /// \brief Flag to check if entities are initialized
  bool entities_initialized_{false};
};

/// \brief Plugin that publishes 3D pose and twist ground truth
///
/// This plugin publishes the pose and twist of a specified link
/// as nav_msgs/Odometry messages for ground truth comparison.
///
/// ## System Parameters
///
/// - `<robot_namespace>` - Namespace for the ROS node (optional)
/// - `<gazebo_child_frame>` - Gazebo link name to track (required)
/// - `<gazebo_frame>` - Gazebo reference frame (default: "world")
/// - `<topic_name>` - ROS topic name for publishing (required)
/// - `<ros_frame_id>` - ROS odometry parent frame_id (optional, default: "odom")
/// - `<ros_child_frame_id>` - ROS odometry child_frame_id (optional, default: gazebo_child_frame)
/// - `<local_twist>` - Compute twist in local frame (default: false)
/// - `<xyz_offset>` - XYZ position offset (default: 0 0 0)
/// - `<rpy_offset>` - RPY orientation offset (default: 0 0 0)
/// - `<gaussian_noise>` - Gaussian noise std dev (default: 0.0)
/// - `<update_rate>` - Update rate in Hz (default: 0 = unlimited)
///
/// ## Example Usage
/// ```xml
/// <plugin filename="GlobalOdometryPublisher" name="ignition::gazebo::systems::GlobalOdometryPublisher">
///   <gazebo_child_frame>base_link</gazebo_child_frame>
///   <gazebo_frame>world</gazebo_frame>
///   <topic_name>ground_truth/odom</topic_name>
///   <ros_frame_id>odom</ros_frame_id>
///   <ros_child_frame_id>base_footprint</ros_child_frame_id>
///   <update_rate>30</update_rate>
///   <gaussian_noise>0.01</gaussian_noise>
/// </plugin>
/// ```
class GlobalOdometryPublisher : public ignition::gazebo::System,
                                public ignition::gazebo::ISystemConfigure,
                                public ignition::gazebo::ISystemPreUpdate,
                                public ignition::gazebo::ISystemPostUpdate
{
public:
  /// \brief Constructor
  GlobalOdometryPublisher();

  /// \brief Destructor
  ~GlobalOdometryPublisher() override;

  /// \brief Configure the system
  /// \param[in] _entity The entity this plugin is attached to
  /// \param[in] _sdf The SDF Element associated with this system plugin
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  ///                  instance
  /// \param[in] _eventMgr The EventManager of the given simulation instance
  void Configure(
    const ignition::gazebo::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    ignition::gazebo::EntityComponentManager & _ecm,
    ignition::gazebo::EventManager & _eventMgr) override;

  /// \brief Called at each simulation iteration (before physics update)
  /// \param[in] _info Current simulation information
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  ///                  instance
  void PreUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    ignition::gazebo::EntityComponentManager & _ecm) override;

  /// \brief Called at each simulation iteration (after physics update)
  /// \param[in] _info Current simulation information
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  ///                  instance
  void PostUpdate(
    const ignition::gazebo::UpdateInfo & _info,
    const ignition::gazebo::EntityComponentManager & _ecm) override;

private:
  /// \brief Generate Gaussian random noise
  /// \param[in] mu Mean
  /// \param[in] sigma Standard deviation
  /// \return Random value from Gaussian distribution
  double gaussianKernel(double mu, double sigma);

  /// \brief Private data pointer
  std::unique_ptr<GlobalOdometryPublisherPrivate> data_ptr_;
};

}  // namespace scorpio_simulator

#endif  // SCORPIO_SIMULATOR__GLOBAL_ODOMETRY_PUBLISHER_HPP_
