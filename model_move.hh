#ifndef _MODEL_MOVE_HH_
#define _MODEL_MOVE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
  class ModelMove : public ModelPlugin
  {
  public:
    ModelMove();
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    // Method to set speed multiplier
    void SetSpeed(float _speed);

  private:
    void Move(const ignition::math::Vector3d &_start,
              const ignition::math::Vector3d &_end,
              ignition::math::Vector3d &_translation);

    void InitiateMove();
    void OnPathMsg(ConstPoseAnimationPtr &_msg);
    bool LoadGoalsFromSDF(const sdf::ElementPtr _sdf);

    physics::ModelPtr model;
    transport::NodePtr node;
    transport::SubscriberPtr pathSubscriber;
    ignition::math::Vector3d startPosition;
    std::vector<ignition::math::Pose3d> pathGoals;
    gazebo::common::PoseAnimationPtr anim;

    // Speed multiplier
    float speed;
  };
}
#endif
