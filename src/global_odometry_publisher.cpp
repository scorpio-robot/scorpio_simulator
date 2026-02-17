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
#include <sstream>
#include <string>

#include "gz/common/Console.hh"
#include "gz/msgs/Utility.hh"
#include "gz/msgs/odometry.pb.h"
#include "gz/msgs/odometry_with_covariance.pb.h"
#include "gz/msgs/pose_v.pb.h"
#include "gz/sim/Model.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Pose.hh"

namespace scorpio_simulator
{

GlobalOdometryPublisher::GlobalOdometryPublisher()
: data_ptr_(std::make_unique<GlobalOdometryPublisherPrivate>())
{
}

GlobalOdometryPublisher::~GlobalOdometryPublisher() {}

void GlobalOdometryPublisher::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & /*_eventMgr*/)
{
  // Get model
  data_ptr_->model_entity_ = _entity;
  auto model = gz::sim::Model(_entity);

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

  // Get topic name (optional)
  if (_sdf->HasElement("topic_name")) {
    data_ptr_->topic_name_ = _sdf->Get<std::string>("topic_name");
  } else {
    data_ptr_->topic_name_ =
      "/model/" + model.Name(_ecm) + "/" + data_ptr_->gazebo_child_frame_ + "/odometry";
    igndbg << "GlobalOdometryPublisher plugin missing <topic_name>, defaults to "
           << data_ptr_->topic_name_ << '\n';
  }

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

  // Get publish_tf flag (optional, defaults to false)
  if (_sdf->HasElement("publish_tf")) {
    data_ptr_->publish_tf_ = _sdf->Get<bool>("publish_tf");
  } else {
    data_ptr_->publish_tf_ = false;
    igndbg << "GlobalOdometryPublisher plugin missing <publish_tf>, defaults to false" << '\n';
  }

  // Get virtual world origin (optional, defaults to identity pose)
  if (_sdf->HasElement("virtual_world_origin")) {
    std::string origin_str = _sdf->Get<std::string>("virtual_world_origin");
    std::istringstream iss(origin_str);
    double x, y, z, roll, pitch, yaw;
    if (iss >> x >> y >> z >> roll >> pitch >> yaw) {
      gz::math::Vector3d pos(x, y, z);
      gz::math::Quaterniond rot(roll, pitch, yaw);
      data_ptr_->virtual_world_origin_ = gz::math::Pose3d(pos, rot);
      igndbg << "GlobalOdometryPublisher plugin virtual_world_origin set to: " << x << " " << y
             << " " << z << " " << roll << " " << pitch << " " << yaw << '\n';
    } else {
      ignerr << "Failed to parse virtual_world_origin. Expected format: x y z roll pitch yaw"
             << '\n';
      data_ptr_->virtual_world_origin_ = gz::math::Pose3d::Zero;
    }
  } else {
    data_ptr_->virtual_world_origin_ = gz::math::Pose3d::Zero;
    igndbg << "GlobalOdometryPublisher plugin missing <virtual_world_origin>, defaults to identity"
           << '\n';
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

  // Create publishers
  std::string odom_topic = data_ptr_->topic_name_;

  // Validate topic
  std::string valid_topic = gz::transport::TopicUtils::AsValidTopic(odom_topic);
  if (valid_topic.empty()) {
    ignerr << "Invalid topic name: " << odom_topic << '\n';
    return;
  }

  // Advertise Odometry
  data_ptr_->pub_ = data_ptr_->node_.Advertise<gz::msgs::Odometry>(valid_topic);

  // Advertise OdometryWithCovariance
  std::string cov_topic = valid_topic + "_with_covariance";
  data_ptr_->pub_cov_ = data_ptr_->node_.Advertise<gz::msgs::OdometryWithCovariance>(cov_topic);

  // Advertise TF if requested
  if (data_ptr_->publish_tf_) {
    std::string tf_topic =
      "/model/" + model.Name(_ecm) + "/" + data_ptr_->gazebo_child_frame_ + "/pose";
    if (_sdf->HasElement("tf_topic")) {
      tf_topic = _sdf->Get<std::string>("tf_topic");
    }
    data_ptr_->pub_tf_ = data_ptr_->node_.Advertise<gz::msgs::Pose_V>(tf_topic);
  }

  // Initialize Gaussian noise generator
  data_ptr_->normal_distribution_ =
    std::normal_distribution<double>(0.0, data_ptr_->gaussian_noise_);

  igndbg << "GlobalOdometryPublisher plugin initialized:" << '\n'
         << "  Gazebo frames: " << data_ptr_->gazebo_frame_ << " -> "
         << data_ptr_->gazebo_child_frame_ << '\n'
         << "  ROS frames: " << data_ptr_->ros_frame_id_ << " -> " << data_ptr_->ros_child_frame_id_
         << '\n'
         << "  Topic: " << valid_topic << '\n';
}

void GlobalOdometryPublisher::PreUpdate(
  const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  // Initialize entities and components on first update
  if (!data_ptr_->entities_initialized_) {
    auto model = gz::sim::Model(data_ptr_->model_entity_);

    // Find link entity
    data_ptr_->link_entity_ = model.LinkByName(_ecm, data_ptr_->gazebo_child_frame_);
    if (data_ptr_->link_entity_ == gz::sim::kNullEntity) {
      ignerr << "GlobalOdometryPublisher: Link [" << data_ptr_->gazebo_child_frame_ << "] not found"
             << '\n';
      return;
    }

    // Find reference link if frame is not world/map
    if (
      data_ptr_->gazebo_frame_ != "world" && data_ptr_->gazebo_frame_ != "map" &&
      data_ptr_->gazebo_frame_ != "/world" && data_ptr_->gazebo_frame_ != "/map") {
      data_ptr_->reference_link_entity_ = model.LinkByName(_ecm, data_ptr_->gazebo_frame_);
      if (data_ptr_->reference_link_entity_ == gz::sim::kNullEntity) {
        ignerr << "GlobalOdometryPublisher: Reference link [" << data_ptr_->gazebo_frame_
               << "] not found, will not publish pose" << '\n';
        return;
      }
    }

    // Create components for link if they don't exist
    if (!_ecm.Component<gz::sim::components::WorldPose>(data_ptr_->link_entity_)) {
      _ecm.CreateComponent(data_ptr_->link_entity_, gz::sim::components::WorldPose());
    }
    if (!_ecm.Component<gz::sim::components::LinearVelocity>(data_ptr_->link_entity_)) {
      _ecm.CreateComponent(data_ptr_->link_entity_, gz::sim::components::LinearVelocity());
    }
    if (!_ecm.Component<gz::sim::components::AngularVelocity>(data_ptr_->link_entity_)) {
      _ecm.CreateComponent(data_ptr_->link_entity_, gz::sim::components::AngularVelocity());
    }

    // Create components for reference link if needed
    if (data_ptr_->reference_link_entity_ != gz::sim::kNullEntity) {
      if (!_ecm.Component<gz::sim::components::WorldPose>(data_ptr_->reference_link_entity_)) {
        _ecm.CreateComponent(data_ptr_->reference_link_entity_, gz::sim::components::WorldPose());
      }
      if (!_ecm.Component<gz::sim::components::LinearVelocity>(data_ptr_->reference_link_entity_)) {
        _ecm.CreateComponent(
          data_ptr_->reference_link_entity_, gz::sim::components::LinearVelocity());
      }
      if (!_ecm.Component<gz::sim::components::AngularVelocity>(
            data_ptr_->reference_link_entity_)) {
        _ecm.CreateComponent(
          data_ptr_->reference_link_entity_, gz::sim::components::AngularVelocity());
      }
    }

    data_ptr_->entities_initialized_ = true;
    data_ptr_->last_update_time_ = _info.simTime;
  }
}

void GlobalOdometryPublisher::PostUpdate(
  const gz::sim::UpdateInfo & _info, const gz::sim::EntityComponentManager & _ecm)
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
  auto world_pose_comp = _ecm.Component<gz::sim::components::WorldPose>(data_ptr_->link_entity_);
  auto world_lin_vel_comp =
    _ecm.Component<gz::sim::components::LinearVelocity>(data_ptr_->link_entity_);
  auto world_ang_vel_comp =
    _ecm.Component<gz::sim::components::AngularVelocity>(data_ptr_->link_entity_);

  if (!world_pose_comp || !world_lin_vel_comp || !world_ang_vel_comp) {
    return;
  }

  gz::math::Pose3d pose = world_pose_comp->Data();
  gz::math::Vector3d vpos = world_lin_vel_comp->Data();
  gz::math::Vector3d veul = world_ang_vel_comp->Data();

  // Apply reference frame if specified
  if (data_ptr_->reference_link_entity_ != gz::sim::kNullEntity) {
    auto ref_pose_comp =
      _ecm.Component<gz::sim::components::WorldPose>(data_ptr_->reference_link_entity_);
    auto ref_lin_vel_comp =
      _ecm.Component<gz::sim::components::LinearVelocity>(data_ptr_->reference_link_entity_);
    auto ref_ang_vel_comp =
      _ecm.Component<gz::sim::components::AngularVelocity>(data_ptr_->reference_link_entity_);

    if (ref_pose_comp && ref_lin_vel_comp && ref_ang_vel_comp) {
      gz::math::Pose3d frame_pose = ref_pose_comp->Data();
      gz::math::Vector3d frame_vpos = ref_lin_vel_comp->Data();
      gz::math::Vector3d frame_veul = ref_ang_vel_comp->Data();

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

  // Transform pose to virtual world coordinate system
  // pose_virtual = virtual_world_origin^-1 * pose_real
  pose = data_ptr_->virtual_world_origin_.Inverse() * pose;

  // Transform velocities to virtual world coordinate system
  vpos = data_ptr_->virtual_world_origin_.Rot().Inverse().RotateVector(vpos);
  veul = data_ptr_->virtual_world_origin_.Rot().Inverse().RotateVector(veul);

  // Create and publish odometry message
  gz::msgs::Odometry odom_msg;

  // Set header
  auto header = odom_msg.mutable_header();
  header->mutable_stamp()->CopyFrom(gz::msgs::Convert(_info.simTime));

  auto frame_id = header->add_data();
  frame_id->set_key("frame_id");
  frame_id->add_value(data_ptr_->ros_frame_id_);

  auto child_frame_id = header->add_data();
  child_frame_id->set_key("child_frame_id");
  child_frame_id->add_value(data_ptr_->ros_child_frame_id_);

  // Set pose
  gz::msgs::Set(odom_msg.mutable_pose(), pose);

  // Set twist with Gaussian noise
  odom_msg.mutable_twist()->mutable_linear()->set_x(
    vpos.X() + gaussianKernel(0, data_ptr_->gaussian_noise_));
  odom_msg.mutable_twist()->mutable_linear()->set_y(
    vpos.Y() + gaussianKernel(0, data_ptr_->gaussian_noise_));
  odom_msg.mutable_twist()->mutable_linear()->set_z(
    vpos.Z() + gaussianKernel(0, data_ptr_->gaussian_noise_));

  odom_msg.mutable_twist()->mutable_angular()->set_x(
    veul.X() + gaussianKernel(0, data_ptr_->gaussian_noise_));
  odom_msg.mutable_twist()->mutable_angular()->set_y(
    veul.Y() + gaussianKernel(0, data_ptr_->gaussian_noise_));
  odom_msg.mutable_twist()->mutable_angular()->set_z(
    veul.Z() + gaussianKernel(0, data_ptr_->gaussian_noise_));

  // Publish message
  if (data_ptr_->pub_.Valid()) {
    data_ptr_->pub_.Publish(odom_msg);
  }

  // Publish OdometryWithCovariance
  if (data_ptr_->pub_cov_.Valid()) {
    gz::msgs::OdometryWithCovariance odom_cov_msg;
    odom_cov_msg.mutable_header()->CopyFrom(*header);

    // Copy pose and twist
    odom_cov_msg.mutable_pose_with_covariance()->mutable_pose()->CopyFrom(odom_msg.pose());
    odom_cov_msg.mutable_twist_with_covariance()->mutable_twist()->CopyFrom(odom_msg.twist());

    // Set covariance
    double gn2 = data_ptr_->gaussian_noise_ * data_ptr_->gaussian_noise_;
    for (int i = 0; i < 36; i++) {
      if (i % 7 == 0) {
        odom_cov_msg.mutable_pose_with_covariance()->mutable_covariance()->add_data(gn2);
        odom_cov_msg.mutable_twist_with_covariance()->mutable_covariance()->add_data(gn2);
      } else {
        odom_cov_msg.mutable_pose_with_covariance()->mutable_covariance()->add_data(0.0);
        odom_cov_msg.mutable_twist_with_covariance()->mutable_covariance()->add_data(0.0);
      }
    }
    data_ptr_->pub_cov_.Publish(odom_cov_msg);
  }

  // Publish TF
  if (data_ptr_->pub_tf_.Valid()) {
    gz::msgs::Pose_V tf_msg;
    auto tf_pose = tf_msg.add_pose();
    tf_pose->CopyFrom(odom_msg.pose());
    tf_pose->mutable_header()->CopyFrom(*header);
    data_ptr_->pub_tf_.Publish(tf_msg);
  }

  // Update last time
  data_ptr_->last_update_time_ = _info.simTime;
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
#include "gz/plugin/Register.hh"
GZ_ADD_PLUGIN(
  scorpio_simulator::GlobalOdometryPublisher, gz::sim::System,
  scorpio_simulator::GlobalOdometryPublisher::ISystemConfigure,
  scorpio_simulator::GlobalOdometryPublisher::ISystemPreUpdate,
  scorpio_simulator::GlobalOdometryPublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  scorpio_simulator::GlobalOdometryPublisher, "gz::sim::systems::GlobalOdometryPublisher")
