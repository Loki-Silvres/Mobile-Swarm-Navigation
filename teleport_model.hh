#ifndef _TELEPORT_MODEL_HH_
#define _TELEPORT_MODEL_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <vector>

namespace gazebo
{
  class TeleportModel : public ModelPlugin
  {
  public:
    TeleportModel();
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  private:
    void TeleportToNextGoal();
    bool LoadGoalsFromSDF(const sdf::ElementPtr _sdf);

    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    ignition::math::Vector3d startPosition;
    std::vector<ignition::math::Pose3d> teleportGoals;
    size_t currentGoalIndex;
    double teleportInterval;
    common::Time lastUpdateTime;
  };
}

#endif
