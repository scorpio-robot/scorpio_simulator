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

#include "scorpio_simulator/global_odometry_publisher.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "ignition/common/Console.hh"
#include "ignition/common/Profiler.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/plugin/Register.hh"

namespace scorpio_simulator
{

GlobalOdometryPublisher::GlobalOdometryPublisher()
: data_ptr_(std::make_unique<GlobalOdometryPublisherPrivate>())
{
}

GlobalOdometryPublisher::~GlobalOdometryPublisher()
{
  // Shutdown ROS node
  if (data_ptr_->ros_node_) {
    data_ptr_->ros_node_.reset();
  }
}

void GlobalOdometryPublisher::Configure(
  const ignition::gazebo::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm, ignition::gazebo::EventManager & /*_eventMgr*/)
{
  // Get model
  data_ptr_->model_entity_ = _entity;
  auto model = ignition::gazebo::Model(_entity);

  if (!model.Valid(_ecm)) {
    ignerr << "GlobalOdometryPublisher plugin should be attached to a model entity. "
           << "Failed to initialize." << '\n';
    return;
  }

  // Get robot namespace
  data_ptr_->robot_namespace_ = "";
  if (_sdf->HasElement("robot_namespace")) {
    data_ptr_->robot_namespace_ = _sdf->Get<std::string>("robot_namespace");
  }

  // Get gazebo child frame (required) - the link to track
  if (!_sdf->HasElement("gazebo_child_frame")) {
    ignerr << "GlobalOdometryPublisher plugin missing <gazebo_child_frame>, cannot proceed" << '\n';
    return;
  }
  data_ptr_->gazebo_child_frame_ = _sdf->Get<std::string>("gazebo_child_frame");

  // Get topic name (required)
  if (!_sdf->HasElement("topic_name")) {
    ignerr << "GlobalOdometryPublisher plugin missing <topic_name>, cannot proceed" << '\n';
    return;
  }
  data_ptr_->topic_name_ = _sdf->Get<std::string>("topic_name");

  // Get gazebo parent frame (optional, defaults to "world")
  if (_sdf->HasElement("gazebo_frame")) {
    data_ptr_->gazebo_frame_ = _sdf->Get<std::string>("gazebo_frame");
  } else {
    data_ptr_->gazebo_frame_ = "world";
    igndbg << "GlobalOdometryPublisher plugin missing <gazebo_frame>, defaults to world" << '\n';
  }

  // Get local twist flag (optional, defaults to false)
  if (_sdf->HasElement("local_twist")) {
    data_ptr_->local_twist_ = _sdf->Get<bool>("local_twist");
  } else {
    data_ptr_->local_twist_ = false;
    igndbg << "GlobalOdometryPublisher plugin missing <local_twist>, defaults to false" << '\n';
  }

  // Get XYZ offset (optional, defaults to [0, 0, 0])
  if (_sdf->HasElement("xyz_offset")) {
    data_ptr_->offset_.Pos() = _sdf->Get<ignition::math::Vector3d>("xyz_offset");
  } else {
    data_ptr_->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
    igndbg << "GlobalOdometryPublisher plugin missing <xyz_offset>, defaults to 0s" << '\n';
  }

  // Get RPY offset (optional, defaults to [0, 0, 0])
  if (_sdf->HasElement("rpy_offset")) {
    auto rpy = _sdf->Get<ignition::math::Vector3d>("rpy_offset");
    data_ptr_->offset_.Rot() = ignition::math::Quaterniond(rpy);
  } else {
    data_ptr_->offset_.Rot() = ignition::math::Quaterniond(0, 0, 0);
    igndbg << "GlobalOdometryPublisher plugin missing <rpy_offset>, defaults to 0s" << '\n';
  }

  // Get Gaussian noise (optional, defaults to 0.0)
  if (_sdf->HasElement("gaussian_noise")) {
    data_ptr_->gaussian_noise_ = _sdf->Get<double>("gaussian_noise");
  } else {
    data_ptr_->gaussian_noise_ = 0.0;
    igndbg << "GlobalOdometryPublisher plugin missing <gaussian_noise>, defaults to 0.0" << '\n';
  }

  // Get update rate (optional, defaults to 0.0 = unlimited)
  if (_sdf->HasElement("update_rate")) {
    data_ptr_->update_rate_ = _sdf->Get<double>("update_rate");
  } else {
    data_ptr_->update_rate_ = 0.0;
    igndbg << "GlobalOdometryPublisher plugin missing <update_rate>, defaults to 0.0 "
           << "(as fast as possible)" << '\n';
  }

  // Initialize ROS node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  std::string node_name = "global_odometry_publisher";
  if (!data_ptr_->robot_namespace_.empty()) {
    node_name = data_ptr_->robot_namespace_ + "_" + node_name;
  }

  data_ptr_->ros_node_ = rclcpp::Node::make_shared(node_name);

  // Get ROS frame_id (optional, defaults to "odom")
  if (_sdf->HasElement("ros_frame_id")) {
    data_ptr_->ros_frame_id_ = _sdf->Get<std::string>("ros_frame_id");
  } else {
    data_ptr_->ros_frame_id_ = "odom";
    igndbg << "GlobalOdometryPublisher plugin missing <ros_frame_id>, defaults to odom" << '\n';
  }

  // Get ROS child_frame_id (optional, defaults to gazebo_child_frame)
  if (_sdf->HasElement("ros_child_frame_id")) {
    data_ptr_->ros_child_frame_id_ = _sdf->Get<std::string>("ros_child_frame_id");
  } else {
    data_ptr_->ros_child_frame_id_ = data_ptr_->gazebo_child_frame_;
    igndbg << "GlobalOdometryPublisher plugin missing <ros_child_frame_id>, defaults to "
              "gazebo_child_frame: "
           << data_ptr_->ros_child_frame_id_ << '\n';
  }

  // Create publisher
  data_ptr_->pub_ =
    data_ptr_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(data_ptr_->topic_name_, 10);

  // Initialize Gaussian noise generator
  data_ptr_->normal_distribution_ =
    std::normal_distribution<double>(0.0, data_ptr_->gaussian_noise_);

  igndbg << "GlobalOdometryPublisher plugin initialized:" << '\n'
         << "  Gazebo frames: " << data_ptr_->gazebo_frame_ << " -> "
         << data_ptr_->gazebo_child_frame_ << '\n'
         << "  ROS frames: " << data_ptr_->ros_frame_id_ << " -> " << data_ptr_->ros_child_frame_id_
         << '\n'
         << "  Topic: " << data_ptr_->topic_name_ << '\n';
}

void GlobalOdometryPublisher::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info, ignition::gazebo::EntityComponentManager & _ecm)
{
  // Initialize entities and components on first update
  if (!data_ptr_->entities_initialized_) {
    auto model = ignition::gazebo::Model(data_ptr_->model_entity_);

    // Find link entity
    data_ptr_->link_entity_ = model.LinkByName(_ecm, data_ptr_->gazebo_child_frame_);
    if (data_ptr_->link_entity_ == ignition::gazebo::kNullEntity) {
      ignerr << "GlobalOdometryPublisher: Link [" << data_ptr_->gazebo_child_frame_ << "] not found"
             << '\n';
      return;
    }

    // Find reference link if frame is not world/map
    if (
      data_ptr_->gazebo_frame_ != "world" && data_ptr_->gazebo_frame_ != "map" &&
      data_ptr_->gazebo_frame_ != "/world" && data_ptr_->gazebo_frame_ != "/map") {
      data_ptr_->reference_link_entity_ = model.LinkByName(_ecm, data_ptr_->gazebo_frame_);
      if (data_ptr_->reference_link_entity_ == ignition::gazebo::kNullEntity) {
        ignerr << "GlobalOdometryPublisher: Reference link [" << data_ptr_->gazebo_frame_
               << "] not found, will not publish pose" << '\n';
        return;
      }
    }

    // Create components for link if they don't exist
    if (!_ecm.Component<ignition::gazebo::components::WorldPose>(data_ptr_->link_entity_)) {
      _ecm.CreateComponent(data_ptr_->link_entity_, ignition::gazebo::components::WorldPose());
    }
    if (!_ecm.Component<ignition::gazebo::components::LinearVelocity>(data_ptr_->link_entity_)) {
      _ecm.CreateComponent(data_ptr_->link_entity_, ignition::gazebo::components::LinearVelocity());
    }
    if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(data_ptr_->link_entity_)) {
      _ecm.CreateComponent(
        data_ptr_->link_entity_, ignition::gazebo::components::AngularVelocity());
    }

    // Create components for reference link if needed
    if (data_ptr_->reference_link_entity_ != ignition::gazebo::kNullEntity) {
      if (!_ecm.Component<ignition::gazebo::components::WorldPose>(
            data_ptr_->reference_link_entity_)) {
        _ecm.CreateComponent(
          data_ptr_->reference_link_entity_, ignition::gazebo::components::WorldPose());
      }
      if (!_ecm.Component<ignition::gazebo::components::LinearVelocity>(
            data_ptr_->reference_link_entity_)) {
        _ecm.CreateComponent(
          data_ptr_->reference_link_entity_, ignition::gazebo::components::LinearVelocity());
      }
      if (!_ecm.Component<ignition::gazebo::components::AngularVelocity>(
            data_ptr_->reference_link_entity_)) {
        _ecm.CreateComponent(
          data_ptr_->reference_link_entity_, ignition::gazebo::components::AngularVelocity());
      }
    }

    data_ptr_->entities_initialized_ = true;
    data_ptr_->last_update_time_ = _info.simTime;
  }
}

void GlobalOdometryPublisher::PostUpdate(
  const ignition::gazebo::UpdateInfo & _info, const ignition::gazebo::EntityComponentManager & _ecm)
{
  // Check if paused or not initialized
  if (_info.paused || !data_ptr_->entities_initialized_) {
    return;
  }

  // Rate control
  if (data_ptr_->update_rate_ > 0.0) {
    auto dt = _info.simTime - data_ptr_->last_update_time_;
    double dt_sec = std::chrono::duration<double>(dt).count();

    if (dt_sec < (1.0 / data_ptr_->update_rate_)) {
      return;
    }
  }

  // Get link pose and velocities
  auto world_pose_comp =
    _ecm.Component<ignition::gazebo::components::WorldPose>(data_ptr_->link_entity_);
  auto world_lin_vel_comp =
    _ecm.Component<ignition::gazebo::components::LinearVelocity>(data_ptr_->link_entity_);
  auto world_ang_vel_comp =
    _ecm.Component<ignition::gazebo::components::AngularVelocity>(data_ptr_->link_entity_);

  if (!world_pose_comp || !world_lin_vel_comp || !world_ang_vel_comp) {
    return;
  }

  ignition::math::Pose3d pose = world_pose_comp->Data();
  ignition::math::Vector3d vpos = world_lin_vel_comp->Data();
  ignition::math::Vector3d veul = world_ang_vel_comp->Data();

  // Apply reference frame if specified
  if (data_ptr_->reference_link_entity_ != ignition::gazebo::kNullEntity) {
    auto ref_pose_comp =
      _ecm.Component<ignition::gazebo::components::WorldPose>(data_ptr_->reference_link_entity_);
    auto ref_lin_vel_comp = _ecm.Component<ignition::gazebo::components::LinearVelocity>(
      data_ptr_->reference_link_entity_);
    auto ref_ang_vel_comp = _ecm.Component<ignition::gazebo::components::AngularVelocity>(
      data_ptr_->reference_link_entity_);

    if (ref_pose_comp && ref_lin_vel_comp && ref_ang_vel_comp) {
      ignition::math::Pose3d frame_pose = ref_pose_comp->Data();
      ignition::math::Vector3d frame_vpos = ref_lin_vel_comp->Data();
      ignition::math::Vector3d frame_veul = ref_ang_vel_comp->Data();

      // Convert to relative pose
      pose.Pos() = pose.Pos() - frame_pose.Pos();
      pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
      pose.Rot() = pose.Rot() * frame_pose.Rot().Inverse();

      // Convert to relative velocities if not local twist
      if (!data_ptr_->local_twist_) {
        vpos = frame_pose.Rot().RotateVector(vpos - frame_vpos);
        veul = frame_pose.Rot().RotateVector(veul - frame_veul);
      }
    }
  }

  // Get local velocities if requested
  if (data_ptr_->local_twist_ && data_ptr_->reference_link_entity_ == gz::sim::kNullEntity) {
    // Transform world velocities to local frame
    vpos = pose.Rot().RotateVectorReverse(vpos);
    veul = pose.Rot().RotateVectorReverse(veul);
  }

  // Apply constant offsets
  pose.Pos() = pose.Pos() + data_ptr_->offset_.Pos();
  pose.Rot() = data_ptr_->offset_.Rot() * pose.Rot();
  pose.Rot().Normalize();

  // Create and publish odometry message
  nav_msgs::msg::Odometry odom_msg;

  // Set header
  odom_msg.header.stamp = data_ptr_->ros_node_->now();
  odom_msg.header.frame_id = data_ptr_->ros_frame_id_;
  odom_msg.child_frame_id = data_ptr_->ros_child_frame_id_;

  // Set pose
  odom_msg.pose.pose.position.x = pose.Pos().X();
  odom_msg.pose.pose.position.y = pose.Pos().Y();
  odom_msg.pose.pose.position.z = pose.Pos().Z();

  odom_msg.pose.pose.orientation.x = pose.Rot().X();
  odom_msg.pose.pose.orientation.y = pose.Rot().Y();
  odom_msg.pose.pose.orientation.z = pose.Rot().Z();
  odom_msg.pose.pose.orientation.w = pose.Rot().W();

  // Set twist with Gaussian noise
  odom_msg.twist.twist.linear.x = vpos.X() + gaussianKernel(0, data_ptr_->gaussian_noise_);
  odom_msg.twist.twist.linear.y = vpos.Y() + gaussianKernel(0, data_ptr_->gaussian_noise_);
  odom_msg.twist.twist.linear.z = vpos.Z() + gaussianKernel(0, data_ptr_->gaussian_noise_);

  odom_msg.twist.twist.angular.x = veul.X() + gaussianKernel(0, data_ptr_->gaussian_noise_);
  odom_msg.twist.twist.angular.y = veul.Y() + gaussianKernel(0, data_ptr_->gaussian_noise_);
  odom_msg.twist.twist.angular.z = veul.Z() + gaussianKernel(0, data_ptr_->gaussian_noise_);

  // Set covariance
  double gn2 = data_ptr_->gaussian_noise_ * data_ptr_->gaussian_noise_;
  odom_msg.pose.covariance[0] = gn2;   // x
  odom_msg.pose.covariance[7] = gn2;   // y
  odom_msg.pose.covariance[14] = gn2;  // z
  odom_msg.pose.covariance[21] = gn2;  // roll
  odom_msg.pose.covariance[28] = gn2;  // pitch
  odom_msg.pose.covariance[35] = gn2;  // yaw

  odom_msg.twist.covariance[0] = gn2;   // vx
  odom_msg.twist.covariance[7] = gn2;   // vy
  odom_msg.twist.covariance[14] = gn2;  // vz
  odom_msg.twist.covariance[21] = gn2;  // wx
  odom_msg.twist.covariance[28] = gn2;  // wy
  odom_msg.twist.covariance[35] = gn2;  // wz

  // Publish message
  data_ptr_->pub_->publish(odom_msg);

  // Update last time
  data_ptr_->last_update_time_ = _info.simTime;

  // Spin ROS node
  rclcpp::spin_some(data_ptr_->ros_node_);
}

double GlobalOdometryPublisher::gaussianKernel(double mu, double sigma)
{
  if (sigma == 0.0) {
    return mu;
  }

  return mu + sigma * data_ptr_->normal_distribution_(data_ptr_->random_generator_);
}

}  // namespace scorpio_simulator

// Register plugin
IGNITION_ADD_PLUGIN(
  scorpio_simulator::GlobalOdometryPublisher, ignition::gazebo::System,
  scorpio_simulator::GlobalOdometryPublisher::ISystemConfigure,
  scorpio_simulator::GlobalOdometryPublisher::ISystemPreUpdate,
  scorpio_simulator::GlobalOdometryPublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  scorpio_simulator::GlobalOdometryPublisher, "ignition::gazebo::systems::GlobalOdometryPublisher")
