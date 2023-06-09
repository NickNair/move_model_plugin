/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef SYSTEM_PLUGIN_MOVEMODEL_HH_
#define SYSTEM_PLUGIN_MOVEMODEL_HH_

#include <chrono>
#include <gz/sim/System.hh>
#include <gz/math/Spline.hh>
#include <gz/math/RotationSpline.hh>

#include <gz/common/Export.hh>
#include "Animation.hh"

namespace move_model
{

  /// \brief Example showing different ways of commanding an actor.
  class MoveModel:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    /// \brief Constructor
    public: MoveModel();

    /// \brief Destructor
    public: ~MoveModel() override;

    /// Documentation inherited
    public: void Configure(const gz::sim::Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) final;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Actor entity
    
    private: gz::sim::Entity entity;

    private: double x;
    private: double y;
    private: double Angle;
    private: PoseAnimation anim{"model_anim",10, true};

    /// \brief Trajectory poses to change, where the key is the number of
    /// seconds from beginning of simulation, and the value is the new
    /// trajectory pose to set.
    private: std::map<double, gz::math::Pose3d> trajPoses;

    /// \brief Last time, in seconds, when the origin was changed.
    private: int lastOriginChange{0};

    /// \brief Last time, in seconds, when the trajectory pose was changed.
    private: int lastTrajPoseChange{0};

    private: PoseAnimation new_animation{"new",10,true};
  };


    
}

#endif
