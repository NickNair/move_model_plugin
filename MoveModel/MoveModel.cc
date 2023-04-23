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
#include "MoveModel.hh"
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(
    move_model::MoveModel,
    gz::sim::System,
    move_model::MoveModel::ISystemConfigure,
    move_model::MoveModel::ISystemPreUpdate)
using namespace move_model;

//////////////////////////////////////////////////
MoveModel::MoveModel()
{
}

//////////////////////////////////////////////////
MoveModel::~MoveModel()
{
}

//////////////////////////////////////////////////
void MoveModel::Configure(const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  this->entity = _entity;
  auto sdfClone = _sdf->Clone();

  // Trajectory pose demo
  for (auto trajPoseElem = sdfClone->GetElement("waypoint");
       trajPoseElem != nullptr;
       trajPoseElem = trajPoseElem->GetNextElement("waypoint"))
  {
    auto pose = trajPoseElem->Get<gz::math::Pose3d>("pose");
    auto time = trajPoseElem->Get<double>("time");

    this->trajPoses[time] = pose;

    PoseKeyFrame *key = this->anim.CreateKeyFrame(time);
    this->anim.Length(time);
    key->Translation(pose.Pos());
    
    key->Rotation(pose.Rot());
  
    std::cout << "Stored trajectory pose change at [" << time
           << "] seconds to pose ["
           << pose << "]" << std::endl;
  }
  
}

//////////////////////////////////////////////////
void MoveModel::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  auto sec = std::chrono::duration_cast<std::chrono::seconds>(
      _info.simTime).count();
  auto milli = std::chrono::duration_cast<std::chrono::milliseconds>(
      _info.simTime).count();

//   std::cout<<sec<<"\n";
  gz::math::Pose3d targetPos;

  _ecm.SetChanged(this->entity, gz::sim::components::Pose::typeId,
        gz::sim::ComponentState::OneTimeChange);



  PoseAnimation anim("pose_test", 5.0, false);
  PoseKeyFrame *key = anim.CreateKeyFrame(0.0);
  anim.Length(10.0);
  key->Translation(gz::math::Vector3d(0, 0, 0));
  
  key->Rotation(gz::math::Quaterniond(0, 0, 0));
  
  key = anim.CreateKeyFrame(10.0);
  key->Translation(gz::math::Vector3d(5, 5, 0));
  
  key->Rotation(gz::math::Quaterniond(0, 0, 1.54));
  
  anim.AddTime(5.0);
  
  this->anim.Time(milli/1e3);
  
  PoseKeyFrame interpolatedKey(2);
  this->anim.InterpolatedKeyFrame(interpolatedKey);
  // std::cout<<"Time:"<<milli/1e3<<" s \n Pose :";
  // std::cout<<interpolatedKey.Translation()<<"\n";
  // std::cout<<interpolatedKey.Rotation()<<"\n";

  // targetPos.Rotation = interpolatedKey.Rotation();

  targetPos = gz::math::Pose3d(interpolatedKey.Translation(),interpolatedKey.Rotation());

  // EXPECT_TRUE(interpolatedKey.Translation() ==
  //     math::Vector3d(3.76, 7.52, 11.28));
  // EXPECT_TRUE(interpolatedKey.Rotation() ==
  //     math::Quaterniond(0.0302776, 0.0785971, 0.109824));

//   std::cout<<"Sed:"<<new_animation.KeyFrameCount()<<"\n";
  
  
 if(!_info.paused)
 {
  auto poseComp = _ecm.Component<gz::sim::components::Pose>(
        this->entity);
    *poseComp = gz::sim::components::Pose(targetPos);
 }

}
