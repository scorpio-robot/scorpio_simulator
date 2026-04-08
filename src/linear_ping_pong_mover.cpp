// Copyright 2026 Lihan Chen
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

#include "scorpio_simulator/linear_ping_pong_mover.hpp"

#include <chrono>
#include <cmath>
#include <memory>

#include "gz/common/Console.hh"
#include "gz/sim/components/Pose.hh"

namespace scorpio_simulator
{

LinearPingPongMover::LinearPingPongMover()
: data_ptr_(std::make_unique<LinearPingPongMoverPrivate>())
{
}

LinearPingPongMover::~LinearPingPongMover() = default;

void LinearPingPongMover::Configure(
  const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & /*_event_mgr*/)
{
  data_ptr_->model_ = gz::sim::Model(_entity);
  if (!data_ptr_->model_.Valid(_ecm)) {
    ignerr << "LinearPingPongMover must be attached to a model." << '\n';
    return;
  }

  if (!_sdf->HasElement("start") || !_sdf->HasElement("end") || !_sdf->HasElement("speed")) {
    ignerr << "LinearPingPongMover requires <start>, <end> and <speed>." << '\n';
    return;
  }

  data_ptr_->start_ = _sdf->Get<gz::math::Vector3d>("start");
  data_ptr_->end_ = _sdf->Get<gz::math::Vector3d>("end");
  data_ptr_->speed_ = std::abs(_sdf->Get<double>("speed"));

  const gz::math::Vector3d delta = data_ptr_->end_ - data_ptr_->start_;
  data_ptr_->segment_length_ = delta.Length();
  if (data_ptr_->segment_length_ < 1e-6) {
    ignerr << "LinearPingPongMover has zero-length segment. start and end must be different."
           << '\n';
    return;
  }

  if (data_ptr_->speed_ < 1e-6) {
    ignerr << "LinearPingPongMover speed must be positive." << '\n';
    return;
  }

  data_ptr_->direction_unit_ = delta / data_ptr_->segment_length_;
  data_ptr_->distance_on_segment_ = 0.0;
  data_ptr_->direction_sign_ = 1;
  data_ptr_->configured_ = true;

  igndbg << "LinearPingPongMover configured for model [" << data_ptr_->model_.Name(_ecm)
         << "] start=" << data_ptr_->start_ << " end=" << data_ptr_->end_
         << " speed=" << data_ptr_->speed_ << " m/s" << '\n';
}

void LinearPingPongMover::PreUpdate(
  const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm)
{
  if (
    !data_ptr_->configured_ || _info.paused ||
    _info.dt <= std::chrono::steady_clock::duration::zero()) {
    return;
  }

  if (!data_ptr_->initialized_) {
    auto pose_comp = _ecm.Component<gz::sim::components::Pose>(data_ptr_->model_.Entity());
    if (pose_comp) {
      data_ptr_->orientation_ = pose_comp->Data().Rot();
    } else {
      data_ptr_->orientation_ = gz::math::Quaterniond::Identity;
    }

    data_ptr_->model_.SetWorldPoseCmd(
      _ecm, gz::math::Pose3d(data_ptr_->start_, data_ptr_->orientation_));
    data_ptr_->initialized_ = true;
    return;
  }

  const double dt_sec = std::chrono::duration<double>(_info.dt).count();
  data_ptr_->distance_on_segment_ +=
    static_cast<double>(data_ptr_->direction_sign_) * data_ptr_->speed_ * dt_sec;

  // Reflect at endpoints to create reciprocal motion.
  while (data_ptr_->distance_on_segment_ > data_ptr_->segment_length_ ||
         data_ptr_->distance_on_segment_ < 0.0) {
    if (data_ptr_->distance_on_segment_ > data_ptr_->segment_length_) {
      data_ptr_->distance_on_segment_ =
        2.0 * data_ptr_->segment_length_ - data_ptr_->distance_on_segment_;
      data_ptr_->direction_sign_ = -1;
    } else if (data_ptr_->distance_on_segment_ < 0.0) {
      data_ptr_->distance_on_segment_ = -data_ptr_->distance_on_segment_;
      data_ptr_->direction_sign_ = 1;
    }
  }

  const gz::math::Vector3d position =
    data_ptr_->start_ + data_ptr_->direction_unit_ * data_ptr_->distance_on_segment_;
  data_ptr_->model_.SetWorldPoseCmd(_ecm, gz::math::Pose3d(position, data_ptr_->orientation_));
}

}  // namespace scorpio_simulator

#include "gz/plugin/Register.hh"
GZ_ADD_PLUGIN(
  scorpio_simulator::LinearPingPongMover, gz::sim::System,
  scorpio_simulator::LinearPingPongMover::ISystemConfigure,
  scorpio_simulator::LinearPingPongMover::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(scorpio_simulator::LinearPingPongMover, "gz::sim::systems::LinearPingPongMover")
