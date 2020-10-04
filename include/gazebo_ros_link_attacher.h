/*
 * Desc: Gazebo link attacher plugin.
 * Author: Sammy Pfeiffer (sam.pfeiffer@pal-robotics.com)
 * Date: 05/04/2016
 */

#pragma once

#include <optional>
#include <ros/ros.h>

#include <sdf/sdf.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_ros_link_attacher/AttachRequest.h>
#include <gazebo_ros_link_attacher/AttachResponse.h>

namespace gazebo
{

class GazeboRosLinkAttacher : public WorldPlugin
{
 public:
  /// \brief Constructor
  GazeboRosLinkAttacher();

  /// \brief Load the controller
  void Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/) override;

  /// \brief Attach with a revolute joint
  bool attach(const std::string &model1, const std::string &link1,
              const std::string &model2, const std::string &link2,
              std::optional<ignition::math::Vector3d> anchor_position);

  /// \brief Detach
  bool detach(const std::string &model1, const std::string &link1,
              const std::string &model2, const std::string &link2);

  /// \brief Internal representation of a fixed joint
  struct FixedJoint
  {
    std::string model1;
    physics::ModelPtr m1;
    std::string link1;
    physics::LinkPtr l1;
    std::string model2;
    physics::ModelPtr m2;
    std::string link2;
    physics::LinkPtr l2;
    physics::JointPtr joint;
  };

  bool
  getJoint(const std::string &model1, const std::string &link1, const std::string &model2, const std::string &link2,
           FixedJoint &joint);

 private:
  ros::NodeHandle nh_;
  ros::ServiceServer attach_service_;
  ros::ServiceServer detach_service_;

  bool attach_callback(gazebo_ros_link_attacher::Attach::Request &req,
                       gazebo_ros_link_attacher::Attach::Response &res);

  bool detach_callback(gazebo_ros_link_attacher::Attach::Request &req,
                       gazebo_ros_link_attacher::Attach::Response &res);

  std::vector<FixedJoint> joints;

  /// \brief The physics engine.
  physics::PhysicsEnginePtr physics;

  /// \brief Pointer to the world.
  physics::WorldPtr world;

};

}