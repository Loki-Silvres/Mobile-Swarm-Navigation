#include "model_move.hh"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelMove)

/////////////////////////////////////////////////
ModelMove::ModelMove() : speed(2.0f)
{
}

/////////////////////////////////////////////////
void ModelMove::SetSpeed(float _speed)
{
    if (_speed > 0)
        this->speed = _speed;
    else
        gzerr << "[model_move] Speed must be positive. Ignoring value: " << _speed << std::endl;
}

/////////////////////////////////////////////////
void ModelMove::Move(const ignition::math::Vector3d &_start, const ignition::math::Vector3d &_end,
                     ignition::math::Vector3d &_translation)
{
    ignition::math::Vector3d direction = _end - _start;
    direction.Normalize();

    int duration = floor(_start.Distance(_end) / this->speed);

    ignition::math::Vector3d diff = _end - _start;
    float xStep = diff.X() / duration;
    float yStep = diff.Y() / duration;
    float zStep = diff.Z() / duration;

    ignition::math::Quaterniond rotation(0, 0, atan2(direction.Y(), direction.X()));

    int currFrame = this->anim->GetKeyFrameCount();

    for (int i = 1; i <= duration; ++i)
    {
        gazebo::common::PoseKeyFrame *key = this->anim->CreateKeyFrame(i + currFrame);
        key->Translation(ignition::math::Vector3d(
             _translation.X() + xStep * i,
             _translation.Y() + yStep * i,
             _translation.Z() + zStep * i));

        key->Rotation(rotation);
    }

    _translation.Set(_translation.X() + xStep * duration,
                     _translation.Y() + yStep * duration,
                     _translation.Z() + zStep * duration);
}

/////////////////////////////////////////////////
void ModelMove::InitiateMove()
{
    float pathLength = this->startPosition.Distance(this->pathGoals[0].Pos());

    for (size_t i = 0; i < this->pathGoals.size() - 1; ++i)
        pathLength += this->pathGoals[i].Pos().Distance(this->pathGoals[i + 1].Pos());

    this->anim = gazebo::common::PoseAnimationPtr(
        new gazebo::common::PoseAnimation("test", (pathLength / this->speed) + 1, true));

    gazebo::common::PoseKeyFrame *key;

    key = this->anim->CreateKeyFrame(0);
    key->Translation(this->startPosition);
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

    ignition::math::Vector3d translation = this->startPosition;

    this->Move(this->startPosition, this->pathGoals[0].Pos(), translation);

    for (size_t i = 0; i < this->pathGoals.size() - 1; ++i)
        this->Move(this->pathGoals[i].Pos(), this->pathGoals[i + 1].Pos(), translation);

    this->model->SetAnimation(this->anim);
}

/////////////////////////////////////////////////
void ModelMove::OnPathMsg(ConstPoseAnimationPtr &_msg)
{
    gzmsg << "[model_move] Received path message" << std::endl;

    for (int i = 0; i < _msg->pose_size(); ++i)
        this->pathGoals.push_back(msgs::ConvertIgn(_msg->pose(i)));

    this->InitiateMove();
}

/////////////////////////////////////////////////
bool ModelMove::LoadGoalsFromSDF(const sdf::ElementPtr _sdf)
{
    gzmsg << "[model_move] Processing path goals defined in the SDF file" << std::endl;

    if (!_sdf->HasElement("pose"))
    {
        gzerr << "[model_move] SDF missing pose elements" << std::endl;
        return false;
    }

    sdf::ElementPtr poseElem = _sdf->GetElement("pose");

    while (poseElem)
    {
        this->pathGoals.push_back(poseElem->Get<ignition::math::Pose3d>());
        poseElem = poseElem->GetNextElement("pose");
    }

    return !this->pathGoals.empty();
}

/////////////////////////////////////////////////
void ModelMove::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    this->model = _parent;
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->Name());

    if (_sdf->HasElement("goals"))
    {
        if (this->LoadGoalsFromSDF(_sdf->GetElement("goals")))
        {
            this->startPosition = this->model->WorldPose().Pos();
            this->InitiateMove();
        }
        else
        {
            gzerr << "[model_move] Failed to load goals from SDF" << std::endl;
        }
    }

    if (_sdf->HasElement("speed"))
    {
        this->speed = _sdf->Get<float>("speed");
        gzmsg << "[model_move] Loaded speed: " << this->speed << std::endl;
        
    }

    std::string pathTopicName = "~/"+ _parent->GetName() + "/model_move";
    this->pathSubscriber = this->node->Subscribe(
        pathTopicName, &ModelMove::OnPathMsg, this);

    gzmsg << "[model_move] Subscribed to topic: " << pathTopicName << std::endl;
}
