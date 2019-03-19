/*
 * Copyright 2016 Open Source Robotics Foundation
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

#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>

#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/AdvertiseOptions.hh>

#include "PriusHybridPlugin.hh"
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include "PriusHybridPlugin.hh"

#include <ros/ros.h>

namespace gazebo
{
  class PriusHybridPluginPrivate
  {

    public: ros::NodeHandle nh;

    public: ros::Subscriber controlSub;

    /// \brief Pointer to the world
    public: physics::WorldPtr world;

    /// \brief Pointer to the parent model
    public: physics::ModelPtr model;

    /// \brief Transport node
    public: transport::NodePtr gznode;

    /// \brief Ignition transport node
    public: ignition::transport::Node node;

    /// \brief Ignition transport position pub
    public: ignition::transport::Node::Publisher posePub;

    /// \brief Ignition transport console pub
    public: ignition::transport::Node::Publisher consolePub;

    /// \brief Physics update event connection
    public: event::ConnectionPtr updateConnection;

    /// \brief Chassis link
    public: physics::LinkPtr chassisLink;

    /// \brief Front left wheel joint
    public: physics::JointPtr flWheelJoint;

    /// \brief Front right wheel joint
    public: physics::JointPtr frWheelJoint;

    /// \brief Rear left wheel joint
    public: physics::JointPtr blWheelJoint;

    /// \brief Rear right wheel joint
    public: physics::JointPtr brWheelJoint;

    /// \brief Front left wheel steering joint
    public: physics::JointPtr flWheelSteeringJoint;

    /// \brief Front right wheel steering joint
    public: physics::JointPtr frWheelSteeringJoint;

    /// \brief Steering wheel joint
    public: physics::JointPtr handWheelJoint;

    /// \brief PID control for the front left wheel steering joint
    public: common::PID flWheelSteeringPID;

    /// \brief PID control for the front right wheel steering joint
    public: common::PID frWheelSteeringPID;

    /// \brief PID control for steering wheel joint
    public: common::PID handWheelPID;

    /// \brief PID control for wheel joint
    public: common::PID flWheelPID;

    /// \brief PID control for wheel joint
    public: common::PID frWheelPID;

    /// \brief PID control for wheel joint
    public: common::PID blWheelPID;

    /// \brief PID control for wheel joint
    public: common::PID brWheelPID;

    /// \brief Last pose msg time
    public: common::Time lastMsgTime;

    /// \brief Last sim time received
    public: common::Time lastSimTime;

    /// \brief Last sim time when a pedal command is received
    public: common::Time lastPedalCmdTime;

    /// \brief Last sim time when a steering command is received
    public: common::Time lastSteeringCmdTime;

    /// \brief Chassis aerodynamic drag force coefficient,
    /// with units of [N / (m/s)^2]
    public: double chassisAeroForceGain = 0;

    /// \brief Max torque that can be applied to the front wheels
    public: double frontTorque = 0;

    /// \brief Max torque that can be applied to the back wheels
    public: double backTorque = 0;

    /// \brief Max speed (m/s) of the car
    public: double maxSpeed = 0;

    /// \brief Max steering angle
    public: double maxSteer = 0;

    /// \brief Max torque that can be applied to the front brakes
    public: double frontBrakeTorque = 0;

    /// \brief Max torque that can be applied to the rear brakes
    public: double backBrakeTorque = 0;

    /// \brief Angle ratio between the steering wheel and the front wheels
    public: double steeringRatio = 0;

    /// \brief Max range of hand steering wheel
    public: double handWheelHigh = 0;

    /// \brief Min range of hand steering wheel
    public: double handWheelLow = 0;

    /// \brief Front left wheel desired steering angle (radians)
    public: double flWheelSteeringCmd = 0;

    /// \brief Front right wheel desired steering angle (radians)
    public: double frWheelSteeringCmd = 0;

    /// \brief Steering wheel desired angle (radians)
    public: double handWheelCmd = 0;

    /// \brief Front left wheel radius
    public: double flWheelRadius = 0;

    /// \brief Front right wheel radius
    public: double frWheelRadius = 0;

    /// \brief Rear left wheel radius
    public: double blWheelRadius = 0;

    /// \brief Rear right wheel radius
    public: double brWheelRadius = 0;

    /// \brief Front left joint friction
    public: double flJointFriction = 0;

    /// \brief Front right joint friction
    public: double frJointFriction = 0;

    /// \brief Rear left joint friction
    public: double blJointFriction = 0;

    /// \brief Rear right joint friction
    public: double brJointFriction = 0;

    /// \brief Distance distance between front and rear axles
    public: double wheelbaseLength = 0;

    /// \brief Distance distance between front left and right wheels
    public: double frontTrackWidth = 0;

    /// \brief Distance distance between rear left and right wheels
    public: double backTrackWidth = 0;
 
    /// \brief Gas pedal position in percentage. 1.0 = Fully accelerated.
    public: double gasPedalPercent = 0;

    /// \brief linear command velocity.
    public: double linearVelCmd = 0;

    /// \brief Front left wheel desired linear velocity (m/s)
    public: double flWheelVelCmd = 0;

    /// \brief Front right wheel desired linear velocity (m/s)
    public: double frWheelVelCmd = 0;

    /// \brief Back left wheel desired linear velocity (m/s)
    public: double blWheelVelCmd = 0;

    /// \brief Back right wheel desired linear velocity (m/s)
    public: double brWheelVelCmd = 0;

    /// \brief Threshold delimiting the gas pedal (throttle) low and medium
    /// ranges.
    public: const double kGasPedalLowMedium = 0.25;

    /// \brief Threshold delimiting the gas pedal (throttle) medium and high
    /// ranges.
    public: const double kGasPedalMediumHigh = 0.5;

    /// \brief Threshold delimiting the speed (throttle) low and medium
    /// ranges in miles/h.
    public: const double speedLowMedium = 26.0;

    /// \brief Threshold delimiting the speed (throttle) medium and high
    /// ranges in miles/h.
    public: const double speedMediumHigh = 46.0;

    /// \brief Brake pedal position in percentage. 1.0 =
    public: double brakePedalPercent = 0;

    /// \brief Hand brake position in percentage.
    public: double handbrakePercent = 1.0;

    /// \brief Angle of steering wheel at last update (radians)
    public: double handWheelAngle = 0;

    /// \brief Steering angle of front left wheel at last update (radians)
    public: double flSteeringAngle = 0;

    /// \brief Steering angle of front right wheel at last update (radians)
    public: double frSteeringAngle = 0;

    /// \brief Linear velocity of chassis c.g. in world frame at last update (m/s)
    public: ignition::math::Vector3d chassisLinearVelocity;

    /// \brief Angular velocity of front left wheel at last update (rad/s)
    public: double flWheelAngularVelocity = 0;

    /// \brief Angular velocity of front right wheel at last update (rad/s)
    public: double frWheelAngularVelocity = 0;

    /// \brief Angular velocity of back left wheel at last update (rad/s)
    public: double blWheelAngularVelocity = 0;

    /// \brief Angular velocity of back right wheel at last update (rad/s)
    public: double brWheelAngularVelocity = 0;

    /// \brief Mutex to protect updates
    public: std::mutex mutex;

    /// \brief Publisher for the world_control topic.
    public: transport::PublisherPtr worldControlPub;

  };
}

using namespace gazebo;

/////////////////////////////////////////////////
PriusHybridPlugin::PriusHybridPlugin()
    : dataPtr(new PriusHybridPluginPrivate)
{
  int argc = 0;
  char *argv = nullptr;
  ros::init(argc, &argv, "PriusHybridPlugin");
  ros::NodeHandle nh;
  this->dataPtr->controlSub = nh.subscribe("cmd_vel", 10, &PriusHybridPlugin::OnPriusCommand, this);

  this->dataPtr->flWheelRadius = 0.3;
  this->dataPtr->frWheelRadius = 0.3;
  this->dataPtr->blWheelRadius = 0.3;
  this->dataPtr->brWheelRadius = 0.3;
}


void PriusHybridPlugin::OnPriusCommand(const geometry_msgs::Twist::ConstPtr &cmd_msg)
{
  this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
  this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();

  // Steering wheel command
  double handCmd = cmd_msg->angular.z / this->dataPtr->steeringRatio;

  // double handCmd = (cmd_msg->angular.z< 0.)
  //   ? (cmd_msg->angular.z * -this->dataPtr->handWheelLow)
  //   : (cmd_msg->angular.z * this->dataPtr->handWheelHigh);

  handCmd = ignition::math::clamp(handCmd, this->dataPtr->handWheelLow,
      this->dataPtr->handWheelHigh);
  this->dataPtr->handWheelCmd = handCmd;

  // double velCmd = cmd_msg->linear.x;
  // this->dataPtr->linearVelCmd = ignition::math::clamp(velCmd, 0.0, this->dataPtr->maxSpeed);
  double velCmd = ignition::math::clamp(cmd_msg->linear.x, 0.0, 1.0);
  this->dataPtr->linearVelCmd = velCmd * this->dataPtr->maxSpeed;

/*  if( cmd_msg->linear.x < 0 )
  {
    // Brake command
    double brake = ignition::math::clamp(-cmd_msg->linear.x, 0.0, 1.0);
    this->dataPtr->brakePedalPercent = brake;
    this->dataPtr->gasPedalPercent = 0;
  }
  else
  {
    // Throttle command
    double throttle = ignition::math::clamp(cmd_msg->linear.x, 0.0, 1.0);
    this->dataPtr->gasPedalPercent = throttle;
    this->dataPtr->brakePedalPercent = 0;
  }*/
}

/////////////////////////////////////////////////
PriusHybridPlugin::~PriusHybridPlugin()
{
  this->dataPtr->updateConnection.reset();
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gzwarn << "PriusHybridPlugin loading params" << std::endl;
  // shortcut to this->dataPtr
  PriusHybridPluginPrivate *dPtr = this->dataPtr.get();

  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();
  auto physicsEngine = this->dataPtr->world->Physics();
  physicsEngine->SetParam("friction_model", std::string("cone_model"));

  this->dataPtr->gznode = transport::NodePtr(new transport::Node());
  this->dataPtr->gznode->Init();

  //this->dataPtr->node.Subscribe("/cmd_vel", &PriusHybridPlugin::OnCmdVel, this);

  this->dataPtr->posePub = this->dataPtr->node.Advertise<ignition::msgs::Pose>(
      "/prius/pose");
  this->dataPtr->consolePub =
    this->dataPtr->node.Advertise<ignition::msgs::Double_V>("/prius/console");

  std::string chassisLinkName = dPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("chassis");
  dPtr->chassisLink = dPtr->model->GetLink(chassisLinkName);
  if (!dPtr->chassisLink)
  {
    std::cerr << "could not find chassis link" << std::endl;
    return;
  }

  std::string handWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("steering_wheel");
  this->dataPtr->handWheelJoint =
    this->dataPtr->model->GetJoint(handWheelJointName);
  if (!this->dataPtr->handWheelJoint)
  {
    std::cerr << "could not find steering wheel joint" <<std::endl;
    return;
  }

  std::string flWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_left_wheel");
  this->dataPtr->flWheelJoint =
    this->dataPtr->model->GetJoint(flWheelJointName);
  if (!this->dataPtr->flWheelJoint)
  {
    std::cerr << "could not find front left wheel joint" <<std::endl;
    return;
  }

  std::string frWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_right_wheel");
  this->dataPtr->frWheelJoint =
    this->dataPtr->model->GetJoint(frWheelJointName);
  if (!this->dataPtr->frWheelJoint)
  {
    std::cerr << "could not find front right wheel joint" <<std::endl;
    return;
  }

  std::string blWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("back_left_wheel");
  this->dataPtr->blWheelJoint =
    this->dataPtr->model->GetJoint(blWheelJointName);
  if (!this->dataPtr->blWheelJoint)
  {
    std::cerr << "could not find back left wheel joint" <<std::endl;
    return;
  }

  std::string brWheelJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("back_right_wheel");
  this->dataPtr->brWheelJoint =
    this->dataPtr->model->GetJoint(brWheelJointName);
  if (!this->dataPtr->brWheelJoint)
  {
    std::cerr << "could not find back right wheel joint" <<std::endl;
    return;
  }

  std::string flWheelSteeringJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_left_wheel_steering");
  this->dataPtr->flWheelSteeringJoint =
    this->dataPtr->model->GetJoint(flWheelSteeringJointName);
  if (!this->dataPtr->flWheelSteeringJoint)
  {
    std::cerr << "could not find front left steering joint" <<std::endl;
    return;
  }

  std::string frWheelSteeringJointName = this->dataPtr->model->GetName() + "::"
    + _sdf->Get<std::string>("front_right_wheel_steering");
  this->dataPtr->frWheelSteeringJoint =
    this->dataPtr->model->GetJoint(frWheelSteeringJointName);
  if (!this->dataPtr->frWheelSteeringJoint)
  {
    std::cerr << "could not find front right steering joint" <<std::endl;
    return;
  }

  std::string paramName;
  double paramDefault;

  paramName = "chassis_aero_force_gain";
  paramDefault = 1;
  if (_sdf->HasElement(paramName))
    this->dataPtr->chassisAeroForceGain = _sdf->Get<double>(paramName);
  else
    this->dataPtr->chassisAeroForceGain = paramDefault;

  paramName = "front_torque";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontTorque = paramDefault;

  paramName = "back_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backTorque = paramDefault;

  paramName = "front_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frontBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->frontBrakeTorque = paramDefault;

  paramName = "back_brake_torque";
  paramDefault = 2000;
  if (_sdf->HasElement(paramName))
    this->dataPtr->backBrakeTorque = _sdf->Get<double>(paramName);
  else
    this->dataPtr->backBrakeTorque = paramDefault;

  paramName = "max_speed";
  paramDefault = 10;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSpeed = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSpeed = paramDefault;

  paramName = "max_steer";
  paramDefault = 0.6;
  if (_sdf->HasElement(paramName))
    this->dataPtr->maxSteer = _sdf->Get<double>(paramName);
  else
    this->dataPtr->maxSteer = paramDefault;

  paramName = "flwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetPGain(paramDefault);

  paramName = "frwheel_steering_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetPGain(paramDefault);

  paramName = "flwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetIGain(paramDefault);

  paramName = "frwheel_steering_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetIGain(paramDefault);

  paramName = "flwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelSteeringPID.SetDGain(paramDefault);

  paramName = "frwheel_steering_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelSteeringPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelSteeringPID.SetDGain(paramDefault);

  paramName = "flwheel_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelPID.SetPGain(paramDefault);

  paramName = "frwheel_p_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelPID.SetPGain(paramDefault);

  paramName = "flwheel_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelPID.SetIGain(paramDefault);

  paramName = "frwheel_i_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelPID.SetIGain(paramDefault);

  paramName = "flwheel_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->flWheelPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->flWheelPID.SetDGain(paramDefault);

  paramName = "frwheel_d_gain";
  paramDefault = 0;
  if (_sdf->HasElement(paramName))
    this->dataPtr->frWheelPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->frWheelPID.SetDGain(paramDefault);

  paramName = "blwheel_p_gain";
  paramDefault = 5;
  if (_sdf->HasElement(paramName))
    this->dataPtr->blWheelPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->blWheelPID.SetPGain(paramDefault);

  paramName = "brwheel_p_gain";
  paramDefault = 5;
  if (_sdf->HasElement(paramName))
    this->dataPtr->brWheelPID.SetPGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->brWheelPID.SetPGain(paramDefault);

  paramName = "blwheel_i_gain";
  paramDefault = 0.1;
  if (_sdf->HasElement(paramName))
    this->dataPtr->blWheelPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->blWheelPID.SetIGain(paramDefault);

  paramName = "brwheel_i_gain";
  paramDefault = 0.1;
  if (_sdf->HasElement(paramName))
    this->dataPtr->brWheelPID.SetIGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->brWheelPID.SetIGain(paramDefault);

  paramName = "blwheel_d_gain";
  paramDefault = 0.1;
  if (_sdf->HasElement(paramName))
    this->dataPtr->blWheelPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->blWheelPID.SetDGain(paramDefault);

  paramName = "brwheel_d_gain";
  paramDefault = 0.1;
  if (_sdf->HasElement(paramName))
    this->dataPtr->brWheelPID.SetDGain(_sdf->Get<double>(paramName));
  else
    this->dataPtr->brWheelPID.SetDGain(paramDefault);

  this->UpdateHandWheelRatio();

  // Update wheel radius for each wheel from SDF collision objects
  //  assumes that wheel link is child of joint (and not parent of joint)
  //  assumes that wheel link has only one collision
  unsigned int id = 0;
  this->dataPtr->flWheelRadius = this->CollisionRadius(
      this->dataPtr->flWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->frWheelRadius = this->CollisionRadius(
      this->dataPtr->frWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->blWheelRadius = this->CollisionRadius(
      this->dataPtr->blWheelJoint->GetChild()->GetCollision(id));
  this->dataPtr->brWheelRadius = this->CollisionRadius(
      this->dataPtr->brWheelJoint->GetChild()->GetCollision(id));

  // Get initial joint friction and add it to braking friction
  dPtr->flJointFriction = dPtr->flWheelJoint->GetParam("friction", 0);
  dPtr->frJointFriction = dPtr->frWheelJoint->GetParam("friction", 0);
  dPtr->blJointFriction = dPtr->blWheelJoint->GetParam("friction", 0);
  dPtr->brJointFriction = dPtr->brWheelJoint->GetParam("friction", 0);

  // Compute wheelbase, frontTrackWidth, and rearTrackWidth
  //  first compute the positions of the 4 wheel centers
  //  again assumes wheel link is child of joint and has only one collision
  ignition::math::Vector3d flCenterPos =
    this->dataPtr->flWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d frCenterPos =
    this->dataPtr->frWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d blCenterPos =
    this->dataPtr->blWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();
  ignition::math::Vector3d brCenterPos =
    this->dataPtr->brWheelJoint->GetChild()->GetCollision(id)
    ->WorldPose().Pos();

  // track widths are computed first
  ignition::math::Vector3d vec3 = flCenterPos - frCenterPos;
  this->dataPtr->frontTrackWidth = vec3.Length();
  vec3 = flCenterPos - frCenterPos;
  this->dataPtr->backTrackWidth = vec3.Length();
  // to compute wheelbase, first position of axle centers are computed
  ignition::math::Vector3d frontAxlePos = (flCenterPos + frCenterPos) / 2;
  ignition::math::Vector3d backAxlePos = (blCenterPos + brCenterPos) / 2;
  // then the wheelbase is the distance between the axle centers
  vec3 = frontAxlePos - backAxlePos;
  this->dataPtr->wheelbaseLength = vec3.Length();

  // gzerr << "wheel base length and track width: "
  //   << this->dataPtr->wheelbaseLength << " "
  //   << this->dataPtr->frontTrackWidth
  //   << " " << this->dataPtr->backTrackWidth << std::endl;

  // Max force that can be applied to hand steering wheel
  double handWheelForce = 10;
  this->dataPtr->handWheelPID.Init(100, 0, 10, 0, 0,
      handWheelForce, -handWheelForce);

  // Max force that can be applied to wheel steering joints
  double kMaxSteeringForceMagnitude = 5000;

  this->dataPtr->flWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  this->dataPtr->flWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  this->dataPtr->frWheelSteeringPID.SetCmdMax(kMaxSteeringForceMagnitude);
  this->dataPtr->frWheelSteeringPID.SetCmdMin(-kMaxSteeringForceMagnitude);

  // Max torque that can be applied to wheel joints
  this->dataPtr->flWheelPID.SetCmdMax(this->dataPtr->frontTorque);
  this->dataPtr->flWheelPID.SetCmdMin(-this->dataPtr->frontBrakeTorque);

  this->dataPtr->frWheelPID.SetCmdMax(this->dataPtr->frontTorque);
  this->dataPtr->frWheelPID.SetCmdMin(-this->dataPtr->frontBrakeTorque);

  this->dataPtr->blWheelPID.SetCmdMax(this->dataPtr->backTorque);
  this->dataPtr->blWheelPID.SetCmdMin(-this->dataPtr->backBrakeTorque);

  this->dataPtr->brWheelPID.SetCmdMax(this->dataPtr->backTorque);
  this->dataPtr->brWheelPID.SetCmdMin(-this->dataPtr->backBrakeTorque);

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&PriusHybridPlugin::Update, this));

  this->dataPtr->worldControlPub =
    this->dataPtr->gznode->Advertise<msgs::WorldControl>("~/world_control");
}

/////////////////////////////////////////////////
//void PriusHybridPlugin::OnCmdVel(const ignition::msgs::Pose &_msg)
//{
//  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
//
//  this->dataPtr->gasPedalPercent = std::min(_msg.position().x(), 1.0);
//  this->dataPtr->handWheelCmd = _msg.position().y();
//  this->dataPtr->brakePedalPercent = _msg.position().z();
//
//  this->dataPtr->lastPedalCmdTime = this->dataPtr->world->SimTime();
//  this->dataPtr->lastSteeringCmdTime = this->dataPtr->world->SimTime();
//}
/////////////////////////////////////////////////
void PriusHybridPlugin::Reset()
{
  this->dataPtr->flWheelSteeringPID.Reset();
  this->dataPtr->frWheelSteeringPID.Reset();
  this->dataPtr->flWheelPID.Reset();
  this->dataPtr->frWheelPID.Reset();
  this->dataPtr->blWheelPID.Reset();
  this->dataPtr->brWheelPID.Reset();
  this->dataPtr->handWheelPID.Reset();
  this->dataPtr->lastMsgTime = 0;
  this->dataPtr->lastSimTime = 0;
  this->dataPtr->lastPedalCmdTime = 0;
  this->dataPtr->lastSteeringCmdTime = 0;
  this->dataPtr->flWheelSteeringCmd = 0;
  this->dataPtr->frWheelSteeringCmd = 0;
  this->dataPtr->handWheelCmd = 0;
  this->dataPtr->linearVelCmd = 0;
  this->dataPtr->brakePedalPercent = 0;
  this->dataPtr->handbrakePercent = 1.0;
  this->dataPtr->handWheelAngle  = 0;
  this->dataPtr->flSteeringAngle = 0;
  this->dataPtr->frSteeringAngle = 0;
  this->dataPtr->flWheelAngularVelocity = 0;
  this->dataPtr->frWheelAngularVelocity = 0;
  this->dataPtr->blWheelAngularVelocity = 0;
  this->dataPtr->brWheelAngularVelocity = 0;
  this->dataPtr->flWheelAngularVelocity  = 0;
  this->dataPtr->frWheelAngularVelocity = 0;
  this->dataPtr->blWheelAngularVelocity = 0;
  this->dataPtr->brWheelAngularVelocity  = 0;
}

/////////////////////////////////////////////////
void PriusHybridPlugin::Update()
{
  // shortcut to this->dataPtr
  PriusHybridPluginPrivate *dPtr = this->dataPtr.get();

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  common::Time curTime = this->dataPtr->world->SimTime();
  double dt = (curTime - this->dataPtr->lastSimTime).Double();
  if (dt < 0)
  {
    this->Reset();
    return;
  }
  else if (ignition::math::equal(dt, 0.0))
  {
    return;
  }

  dPtr->handWheelAngle = dPtr->handWheelJoint->Position();
  dPtr->flSteeringAngle = dPtr->flWheelSteeringJoint->Position();
  dPtr->frSteeringAngle = dPtr->frWheelSteeringJoint->Position();

  dPtr->flWheelAngularVelocity = dPtr->flWheelJoint->GetVelocity(0);
  dPtr->frWheelAngularVelocity = dPtr->frWheelJoint->GetVelocity(0);
  dPtr->blWheelAngularVelocity = dPtr->blWheelJoint->GetVelocity(0);
  dPtr->brWheelAngularVelocity = dPtr->brWheelJoint->GetVelocity(0);

  dPtr->chassisLinearVelocity = dPtr->chassisLink->WorldCoGLinearVel();
  // Convert meter/sec to miles/hour
  double linearVel = dPtr->chassisLinearVelocity.Length() * 2.23694;
  double linearVelocity = dPtr->chassisLinearVelocity.Length();

  this->dataPtr->lastSimTime = curTime;

  // Aero-dynamic drag on chassis
  auto dragForce = -dPtr->chassisAeroForceGain *
        dPtr->chassisLinearVelocity.SquaredLength() *
        dPtr->chassisLinearVelocity.Normalized();
  dPtr->chassisLink->AddForce(dragForce);

  // PID (position) steering
  this->dataPtr->handWheelCmd =
    ignition::math::clamp(this->dataPtr->handWheelCmd,
        -this->dataPtr->maxSteer / this->dataPtr->steeringRatio,
        this->dataPtr->maxSteer / this->dataPtr->steeringRatio);
  double steerError =
      this->dataPtr->handWheelAngle - this->dataPtr->handWheelCmd;
  double steerCmd = this->dataPtr->handWheelPID.Update(steerError, dt);
  this->dataPtr->handWheelJoint->SetForce(0, steerCmd);

  double tanSteer =
      tan(this->dataPtr->handWheelCmd * this->dataPtr->steeringRatio);
  this->dataPtr->flWheelSteeringCmd = atan2(tanSteer,
      1 - this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);
  this->dataPtr->frWheelSteeringCmd = atan2(tanSteer,
      1 + this->dataPtr->frontTrackWidth/2/this->dataPtr->wheelbaseLength *
      tanSteer);

  double flwsError =
      this->dataPtr->flSteeringAngle - this->dataPtr->flWheelSteeringCmd;
  double flwsCmd = this->dataPtr->flWheelSteeringPID.Update(flwsError, dt);
  this->dataPtr->flWheelSteeringJoint->SetForce(0, flwsCmd);

  double frwsError =
      this->dataPtr->frSteeringAngle - this->dataPtr->frWheelSteeringCmd;
  double frwsCmd = this->dataPtr->frWheelSteeringPID.Update(frwsError, dt);
  this->dataPtr->frWheelSteeringJoint->SetForce(0, frwsCmd);


  if (this->dataPtr->frWheelSteeringCmd == 0)
    {
    this->dataPtr->flWheelVelCmd = this->dataPtr->linearVelCmd;
    this->dataPtr->frWheelVelCmd = this->dataPtr->linearVelCmd;
    }
  else
    {
    this->dataPtr->flWheelVelCmd = this->dataPtr->linearVelCmd * tanSteer/sin(this->dataPtr->flWheelSteeringCmd);
    this->dataPtr->frWheelVelCmd = this->dataPtr->linearVelCmd * tanSteer/sin(this->dataPtr->frWheelSteeringCmd);
    }

    this->dataPtr->blWheelVelCmd = this->dataPtr->linearVelCmd * (1.0 - this->dataPtr->backTrackWidth * tanSteer/2/this->dataPtr->wheelbaseLength);
    this->dataPtr->brWheelVelCmd = this->dataPtr->linearVelCmd * (1.0 + this->dataPtr->backTrackWidth * tanSteer/2/this->dataPtr->wheelbaseLength);


  // PID (SetForce) wheel torque
  double flwError = dPtr->flWheelAngularVelocity - this->dataPtr->flWheelVelCmd / this->dataPtr->flWheelRadius; 
  double frwError = dPtr->frWheelAngularVelocity - this->dataPtr->frWheelVelCmd / this->dataPtr->frWheelRadius;
  double blwError = dPtr->blWheelAngularVelocity - this->dataPtr->blWheelVelCmd / this->dataPtr->blWheelRadius; 
  double brwError = dPtr->brWheelAngularVelocity - this->dataPtr->brWheelVelCmd / this->dataPtr->brWheelRadius;

  double flwCmdf = this->dataPtr->flWheelPID.Update(flwError, dt);
  double frwCmdf = this->dataPtr->frWheelPID.Update(frwError, dt);
  double blwCmdf = this->dataPtr->blWheelPID.Update(blwError, dt);
  double brwCmdf = this->dataPtr->brWheelPID.Update(brwError, dt);

  this->dataPtr->flWheelJoint->SetForce(0, flwCmdf);
  this->dataPtr->frWheelJoint->SetForce(0, frwCmdf);
  this->dataPtr->blWheelJoint->SetForce(0, blwCmdf);
  this->dataPtr->brWheelJoint->SetForce(0, brwCmdf);

/*
  // SetVelocity
  double flwCmdv = this->dataPtr->flWheelVelCmd / this->dataPtr->flWheelRadius; 
  double frwCmdv = this->dataPtr->frWheelVelCmd / this->dataPtr->frWheelRadius;
  double blwCmdv = this->dataPtr->blWheelVelCmd / this->dataPtr->blWheelRadius; 
  double brwCmdv = this->dataPtr->brWheelVelCmd / this->dataPtr->brWheelRadius;

  this->dataPtr->flWheelJoint->SetVelocity(0, flwCmdv);
  this->dataPtr->frWheelJoint->SetVelocity(0, flwCmdv);
  this->dataPtr->blWheelJoint->SetVelocity(0, flwCmdv);
  this->dataPtr->brWheelJoint->SetVelocity(0, flwCmdv);
*/
/*
  //static common::Time lastErrorPrintTime = 0.0;
  //if (curTime - lastErrorPrintTime > 0.01 || curTime < lastErrorPrintTime)
  //{
  //  lastErrorPrintTime = curTime;
  //  double maxSteerError =
  //    std::abs(frwsError) > std::abs(flwsError) ? frwsError : flwsError;
  //  double maxSteerErrPer = maxSteerError / this->dataPtr->maxSteer * 100.0;
  //  std::cerr << std::fixed << "Max steering error: " << maxSteerErrPer
  //    << std::endl;
  //}

  // Model low-speed caaaareep and high-speed regen braking
  // with term added to gas/brake
  // Cross-over speed is 7 miles/hour
  // 10% throttle at 0 speed
  // max 2.5% braking at higher speeds
 double creepPercent;
  if (std::abs(linearVel) <= 7)
  {
    creepPercent = 0.1 * (1 - std::abs(linearVel) / 7);
  }
  else
  {
    creepPercent = 0.025 * (7 - std::abs(linearVel));
  }
  creepPercent = ignition::math::clamp(creepPercent, -0.025, 0.1);

  // Gas pedal torque.
  // Map gas torques to individual wheels.
  // Cut off gas torque at a given wheel if max speed is exceeded.
  // Use directionState to determine direction of that can be applied torque.
  // Note that definition of DirectionType allows multiplication to determine
  // torque direction.
  // also, make sure gas pedal is at least as large as the creepPercent.
  double gasPercent = std::max(this->dataPtr->gasPedalPercent, creepPercent);
  double gasMultiplier = 1.0;
  double flGasTorque = 0, frGasTorque = 0, blGasTorque = 0, brGasTorque = 0;
  // Apply equal torque at left and right wheels, which is an implicit model
  // of the differential.
  if (fabs(dPtr->flWheelAngularVelocity * dPtr->flWheelRadius) < dPtr->maxSpeed &&
      fabs(dPtr->frWheelAngularVelocity * dPtr->frWheelRadius) < dPtr->maxSpeed)
  {
    flGasTorque = gasPercent*dPtr->frontTorque * gasMultiplier;
    frGasTorque = gasPercent*dPtr->frontTorque * gasMultiplier;
  }
  if (fabs(dPtr->blWheelAngularVelocity * dPtr->blWheelRadius) < dPtr->maxSpeed &&
      fabs(dPtr->brWheelAngularVelocity * dPtr->brWheelRadius) < dPtr->maxSpeed)
  {
    blGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
    brGasTorque = gasPercent * dPtr->backTorque * gasMultiplier;
  }
  double throttlePower =
      std::abs(flGasTorque * dPtr->flWheelAngularVelocity) +
      std::abs(frGasTorque * dPtr->frWheelAngularVelocity) +
      std::abs(blGasTorque * dPtr->blWheelAngularVelocity) +
      std::abs(brGasTorque * dPtr->brWheelAngularVelocity);

  // auto release handbrake as soon as the gas pedal is depressed
  if (this->dataPtr->gasPedalPercent > 0)
    this->dataPtr->handbrakePercent = 0.0;

  double brakePercent = this->dataPtr->brakePedalPercent
      + this->dataPtr->handbrakePercent;
  // use creep braking if not in Neutral

    brakePercent = std::max(brakePercent,
        -creepPercent - this->dataPtr->gasPedalPercent);

  brakePercent = ignition::math::clamp(brakePercent, 0.0, 1.0);
  dPtr->flWheelJoint->SetParam("friction", 0,
      dPtr->flJointFriction + brakePercent * dPtr->frontBrakeTorque);
  dPtr->frWheelJoint->SetParam("friction", 0,
      dPtr->frJointFriction + brakePercent * dPtr->frontBrakeTorque);
  dPtr->blWheelJoint->SetParam("friction", 0,
      dPtr->blJointFriction + brakePercent * dPtr->backBrakeTorque);
  dPtr->brWheelJoint->SetParam("friction", 0,
      dPtr->brJointFriction + brakePercent * dPtr->backBrakeTorque);

  this->dataPtr->flWheelJoint->SetForce(0, flGasTorque);
  this->dataPtr->frWheelJoint->SetForce(0, frGasTorque);
  this->dataPtr->blWheelJoint->SetForce(0, blGasTorque);
  this->dataPtr->brWheelJoint->SetForce(0, brGasTorque);
*/
    // Output prius car data.
    this->dataPtr->posePub.Publish(
        ignition::msgs::Convert(this->dataPtr->model->WorldPose()));

    this->dataPtr->lastMsgTime = curTime;

  // reset if last command is more than x sec ago
  if ((curTime - this->dataPtr->lastPedalCmdTime).Double() > 0.3)
  {
    this->dataPtr->linearVelCmd = 0;
    //this->dataPtr->gasPedalPercent = 0.0;
    //this->dataPtr->brakePedalPercent = 0.0;
  }

  if ((curTime - this->dataPtr->lastSteeringCmdTime).Double() > 0.3)
  {
    this->dataPtr->handWheelCmd = 0;
  }
}

/////////////////////////////////////////////////
void PriusHybridPlugin::UpdateHandWheelRatio()
{
  // The total range the steering wheel can rotate
  this->dataPtr->handWheelHigh = 7.85;
  this->dataPtr->handWheelLow = -7.85;
  double handWheelRange =
      this->dataPtr->handWheelHigh - this->dataPtr->handWheelLow;
  double high = 0.8727;
  high = std::min(high, this->dataPtr->maxSteer);
  double low = -0.8727;
  low = std::max(low, -this->dataPtr->maxSteer);
  double tireAngleRange = high - low;

  // Compute the angle ratio between the steering wheel and the tires
  this->dataPtr->steeringRatio = tireAngleRange / handWheelRange;
}

/////////////////////////////////////////////////
// function that extracts the radius of a cylinder or sphere collision shape
// the function returns zero otherwise
double PriusHybridPlugin::CollisionRadius(physics::CollisionPtr _coll)
{
  if (!_coll || !(_coll->GetShape()))
    return 0;
  if (_coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
  {
    physics::CylinderShape *cyl =
        static_cast<physics::CylinderShape*>(_coll->GetShape().get());
    return cyl->GetRadius();
  }
  else if (_coll->GetShape()->HasType(physics::Base::SPHERE_SHAPE))
  {
    physics::SphereShape *sph =
        static_cast<physics::SphereShape*>(_coll->GetShape().get());
    return sph->GetRadius();
  }
  return 0;
}

/////////////////////////////////////////////////


GZ_REGISTER_MODEL_PLUGIN(PriusHybridPlugin)
