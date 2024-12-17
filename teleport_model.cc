#include "teleport_model.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TeleportModel)

/////////////////////////////////////////////////
TeleportModel::TeleportModel() : currentGoalIndex(0), teleportInterval(1.0) {}

/////////////////////////////////////////////////
void TeleportModel::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->model = _parent;

  // Load teleportation goals and interval from the SDF file
  if (_sdf->HasElement("teleport_interval"))
    this->teleportInterval = _sdf->Get<double>("teleport_interval");

  if (!_sdf->HasElement("goals") || !this->LoadGoalsFromSDF(_sdf->GetElement("goals")))
  {
    gzerr << "[TeleportModel] No valid teleportation goals provided in SDF." << std::endl;
    return;
  }

  this->startPosition = this->model->WorldPose().Pos();
  this->lastUpdateTime = this->model->GetWorld()->SimTime();

  // Connect to the update event
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&TeleportModel::TeleportToNextGoal, this));
}

/////////////////////////////////////////////////
bool TeleportModel::LoadGoalsFromSDF(const sdf::ElementPtr _sdf)
{
  sdf::ElementPtr poseElem = _sdf->GetElement("pose");
  while (poseElem)
  {
    this->teleportGoals.push_back(poseElem->Get<ignition::math::Pose3d>());
    poseElem = poseElem->GetNextElement("pose");
  }
  return !this->teleportGoals.empty();
}

/////////////////////////////////////////////////
void TeleportModel::TeleportToNextGoal()
{
  auto currentTime = this->model->GetWorld()->SimTime();
  if ((currentTime - this->lastUpdateTime).Double() >= this->teleportInterval)
  {
    // Teleport to the next goal
    this->model->SetWorldPose(this->teleportGoals[this->currentGoalIndex]);

    // Update index and reset timer
    this->currentGoalIndex = (this->currentGoalIndex + 1) % this->teleportGoals.size();
    this->lastUpdateTime = currentTime;
  }
}

