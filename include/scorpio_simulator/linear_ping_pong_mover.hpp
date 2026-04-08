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

#ifndef SCORPIO_SIMULATOR__LINEAR_PING_PONG_MOVER_HPP_
#define SCORPIO_SIMULATOR__LINEAR_PING_PONG_MOVER_HPP_

#include <memory>

#include "gz/math/Pose3.hh"
#include "gz/math/Vector3.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"

namespace scorpio_simulator
{

class LinearPingPongMoverPrivate
{
public:
  gz::sim::Model model_;
  bool configured_{false};
  bool initialized_{false};

  gz::math::Vector3d start_{0.0, 0.0, 0.0};
  gz::math::Vector3d end_{0.0, 0.0, 0.0};
  gz::math::Vector3d direction_unit_{1.0, 0.0, 0.0};

  double segment_length_{0.0};
  double speed_{0.5};
  double distance_on_segment_{0.0};
  int direction_sign_{1};

  gz::math::Quaterniond orientation_;
};

/// \brief Move an attached model linearly between two points and bounce back.
///
/// Required SDF parameters:
/// - <start>x y z</start>
/// - <end>x y z</end>
/// - <speed>m_per_sec</speed>
class LinearPingPongMover : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate
{
public:
  LinearPingPongMover();
  ~LinearPingPongMover() override;

  void Configure(
    const gz::sim::Entity & _entity, const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm, gz::sim::EventManager & _event_mgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info, gz::sim::EntityComponentManager & _ecm) override;

private:
  std::unique_ptr<LinearPingPongMoverPrivate> data_ptr_;
};

}  // namespace scorpio_simulator

#endif  // SCORPIO_SIMULATOR__LINEAR_PING_PONG_MOVER_HPP_
