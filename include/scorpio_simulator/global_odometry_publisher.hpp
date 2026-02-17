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

#include "gz/sim/System.hh"
#include "gz/transport/Node.hh"

namespace scorpio_simulator
{

/// \brief Private data class for GlobalOdometryPublisher
class GlobalOdometryPublisherPrivate
{
public:
  /// \brief Ignition communication node
  gz::transport::Node node_;

  /// \brief Publisher for odometry messages
  gz::transport::Node::Publisher pub_;

  /// \brief Publisher for odometry with covariance messages
  gz::transport::Node::Publisher pub_cov_;

  /// \brief Publisher for TF messages
  gz::transport::Node::Publisher pub_tf_;

  /// \brief Model entity
  gz::sim::Entity model_entity_;

  /// \brief Link entity (body to track)
  gz::sim::Entity link_entity_;

  /// \brief Reference link entity (for relative pose)
  gz::sim::Entity reference_link_entity_;

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

  /// \brief Publish TF
  bool publish_tf_{false};

  /// \brief Compute twist in local frame
  bool local_twist_{false};

  /// \brief Virtual world origin pose (for coordinate system transformation)
  gz::math::Pose3d virtual_world_origin_;

  /// \brief Update rate (Hz), 0 means as fast as possible
  double update_rate_{0.0};

  /// \brief Gaussian noise standard deviation
  double gaussian_noise_{0.0};

  /// \brief Last update time
  std::chrono::steady_clock::duration last_update_time_{0};

  /// \brief Last linear velocity
  gz::math::Vector3d last_vpos_;

  /// \brief Last angular velocity
  gz::math::Vector3d last_veul_;

  /// \brief Random number generator for Gaussian noise
  std::default_random_engine random_generator_;
  std::normal_distribution<double> normal_distribution_;

  /// \brief Flag to check if entities are initialized
  bool entities_initialized_{false};
};

/// \brief Plugin that publishes 3D pose and twist ground truth
///
/// This plugin publishes the pose and twist of a specified link
/// as gz::msgs::Odometry messages for ground truth comparison.
///
/// ## System Parameters
///optional, default: "/model/{model_name}/{child_frame}/odometry")
/// - `<publish_tf>` - Publish TF message (optional, default: false)
/// - `<tf_topic>` - Ignition topic name for publishing TF (optional, default: "/model/{model_name}/{child_fr
/// - `<gazebo_frame>` - Gazebo reference frame (default: "world")
/// - `<topic_name>` - Ignition topic name for publishing Odometry (required)
/// - `<tf_topic>` - Ignition topic name for publishing TF (optional, default: "/model/{model_name}/pose")
/// - `<ros_frame_id>` - Frame ID for ROS message header (optional, default: "odom")
/// - `<ros_child_frame_id>` - Child Frame ID for ROS message header (optional, default: gazebo_child_frame)
/// - `<local_twist>` - Compute twist in local frame (default: false)
/// - `<virtual_world_origin>` - Virtual world origin (x y z roll pitch yaw) (default: 0 0 0 0 0 0)
/// - `<gaussian_noise>` - Gaussian noise std dev (default: 0.0)
/// - `<update_rate>` - Update rate in Hz (default: 0 = unlimited)
///
/// ## Example Usage
/// ```xml
/// <plugin filename="GlobalOdometryPublisher" name="gz::sim::systems::GlobalOdometryPublisher">
///   <gazebo_child_frame>base_link</gazebo_child_frame>
///   <gazebo_frame>world</gazebo_frame>
///   <topic_name>ground_truth/odom</topic_name>
///   <ros_frame_id>odom</ros_frame_id>
///   <ros_child_frame_id>base_footprint</ros_child_frame_id>
///   <update_rate>30</update_rate>
///   <gaussian_noise>0.01</gaussian_noise>
///   <virtual_world_origin>0 0 0 0 0 0</virtual_world_origin>
/// </plugin>
/// ```
class GlobalOdometryPublisher : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate,
                                public gz::sim::ISystemPostUpdate
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
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _eventMgr) override;

  /// \brief Called at each simulation iteration (before physics update)
  /// \param[in] _info Current simulation information
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  ///                  instance
  void PreUpdate(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override;

  /// \brief Called at each simulation iteration (after physics update)
  /// \param[in] _info Current simulation information
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  ///                  instance
  void PostUpdate(
    const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm) override;

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
