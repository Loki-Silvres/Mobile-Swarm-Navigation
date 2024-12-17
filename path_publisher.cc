/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
 */

#include <iostream>
#include <math.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>  // Replacing deprecated gazebo/math

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

using namespace gazebo;

int main(int _argc, char *_argv[])
{
  // Create a PoseAnimation message
  msgs::PoseAnimation msg;

  // Set the model name to be moved
  msg.set_model_name("box");

  // Define the animation poses
  msgs::Pose *p = msg.add_pose();
  msgs::Set(p, ignition::math::Pose3d(5, 5, 0, 0, 0, 0)); // Pose 1
  p = msg.add_pose();
  msgs::Set(p, ignition::math::Pose3d(5, -5, 0, 0, 0, 0)); // Pose 2
  p = msg.add_pose();
  msgs::Set(p, ignition::math::Pose3d(0, 0, 0, 0, 0, 0)); // Pose 3

  // Initialize Gazebo transport
  transport::init();
  transport::run();

  // Create a transport node
  transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");

  // Define the topic to publish the animation
  const std::string topicName = "/gazebo/modelmove_world/" + msg.model_name() + "/model_move";
  gazebo::transport::PublisherPtr pathPub = node->Advertise<msgs::PoseAnimation>(topicName);

  std::cout << "Waiting for connection on " << topicName << std::endl;

  // Wait for connection to the topic
  pathPub->WaitForConnection();

  // Publish the animation message
  pathPub->Publish(msg);

  std::cout << "Path published!" << std::endl;

  // Finalize Gazebo transport
  gazebo::transport::fini();
  return 0;
}
