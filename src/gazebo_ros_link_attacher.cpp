#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo_ros_link_attacher.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosLinkAttacher)

// Constructor
GazeboRosLinkAttacher::GazeboRosLinkAttacher() :
    nh_("link_attacher_node")
{
}

void GazeboRosLinkAttacher::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->world = _world;
  this->physics = this->world->Physics();
  this->attach_service_ = this->nh_.advertiseService("attach", &GazeboRosLinkAttacher::attach_callback, this);
  ROS_INFO_STREAM("Attach service at: " << this->nh_.resolveName("attach"));
  this->detach_service_ = this->nh_.advertiseService("detach", &GazeboRosLinkAttacher::detach_callback, this);
  ROS_INFO_STREAM("Detach service at: " << this->nh_.resolveName("detach"));
  ROS_INFO("Link attacher node initialized.");
}

bool GazeboRosLinkAttacher::attach(const std::string &model1, const std::string &link1,
                                   const std::string &model2, const std::string &link2,
                                   std::optional<ignition::math::Vector3d> anchor_position)
{
  // look for any previous instance of the joint first.
  // if we try to create a joint in between two links
  // more than once (even deleting any reference to the first one)
  // gazebo hangs/crashes
  FixedJoint j;
  if (this->getJoint(model1, link1, model2, link2, j))
  {
    ROS_INFO_STREAM("Joint already existed, reusing it.");
    j.joint->Attach(j.l1, j.l2);
    return true;
  } else
  {
    ROS_INFO_STREAM("Creating new joint.");
  }
  j.model1 = model1;
  j.link1 = link1;
  j.model2 = model2;
  j.link2 = link2;
  ROS_DEBUG_STREAM("Getting BasePtr of " << model1);
  physics::BasePtr b1 = this->world->ModelByName(model1);

  if (!b1)
  {
    ROS_ERROR_STREAM(model1 << " model was not found");
    return false;
  }
  ROS_DEBUG_STREAM("Getting BasePtr of " << model2);
  physics::BasePtr b2 = this->world->ModelByName(model2);
  if (!b2)
  {
    ROS_ERROR_STREAM(model2 << " model was not found");
    return false;
  }

  ROS_DEBUG_STREAM("Casting into ModelPtr");
  physics::ModelPtr m1(dynamic_cast<physics::Model *>(b1.get()));
  j.m1 = m1;
  physics::ModelPtr m2(dynamic_cast<physics::Model *>(b2.get()));
  j.m2 = m2;

  ROS_DEBUG_STREAM("Getting link: '" << link1 << "' from model: '" << model1 << "'");
  physics::LinkPtr l1 = m1->GetLink(link1);
  if (!l1)
  {
    ROS_ERROR_STREAM(link1 << " link was not found");
    return false;
  }
  if (!l1->GetInertial())
  {
    ROS_ERROR_STREAM("link1 inertia is NULL!");
  } else
    ROS_DEBUG_STREAM("link1 inertia is not NULL, for example, mass is: " << l1->GetInertial()->Mass());
  j.l1 = l1;
  ROS_DEBUG_STREAM("Getting link: '" << link2 << "' from model: '" << model2 << "'");
  physics::LinkPtr l2 = m2->GetLink(link2);
  if (!l2)
  {
    ROS_ERROR_STREAM(link2 << " link was not found");
    return false;
  }
  if (!l2->GetInertial())
  {
    ROS_ERROR_STREAM("link2 inertia is NULL!");
  } else
    ROS_DEBUG_STREAM("link2 inertia is not NULL, for example, mass is: " << l2->GetInertial()->Mass());
  j.l2 = l2;

  ROS_DEBUG_STREAM("Links are: " << l1->GetName() << " and " << l2->GetName());

  ROS_DEBUG_STREAM("Creating joint on model: '" << model1 << "'");
  j.joint = this->physics->CreateJoint("fixed", m1);
  std::stringstream ss;
  ss << j.model1 << "_" << j.link1 << "_to_" << j.model2 << "_" << j.link2 << "_joint";
  auto const joint_name = ss.str();
  j.joint->SetName(joint_name);
  this->joints.push_back(j);

  ROS_DEBUG_STREAM("Attach joint with name " << joint_name);
  j.joint->Attach(l1, l2);
  if (anchor_position)
  {
    j.joint->SetAnchor(0, anchor_position.value());
  }
  ROS_DEBUG_STREAM("Loading links");
  j.joint->Load(l1, l2, ignition::math::Pose3d());
  ROS_DEBUG_STREAM("SetModel");
  j.joint->SetModel(m2);
  /*
   * If SetModel is not done we get:
   * ***** Internal Program Error - assertion (this->GetParentModel() != __null)
   failed in void gazebo::physics::Entity::PublishPose():
   /tmp/buildd/gazebo2-2.2.3/gazebo/physics/Entity.cc(225):
   An entity without a parent model should not happen

   * If SetModel is given the same model than CreateJoint given
   * Gazebo crashes with
   * ***** Internal Program Error - assertion (self->inertial != __null)
   failed in static void gazebo::physics::ODELink::MoveCallback(dBodyID):
   /tmp/buildd/gazebo2-2.2.3/gazebo/physics/ode/ODELink.cc(183): Inertial pointer is NULL
   */

  ROS_DEBUG_STREAM("Init");
  j.joint->Init();
  ROS_INFO_STREAM("Attach finished.");

  return true;
}

bool GazeboRosLinkAttacher::detach(const std::string &model1, const std::string &link1,
                                   const std::string &model2, const std::string &link2)
{
  // search for the instance of joint and do detach
  FixedJoint j;
  if (this->getJoint(model1, link1, model2, link2, j))
  {
    j.joint->Detach();
    return true;
  }

  return false;
}

bool GazeboRosLinkAttacher::getJoint(const std::string &model1, const std::string &link1,
                                     const std::string &model2, const std::string &link2,
                                     FixedJoint &joint)
{
  for (const auto &j : this->joints)
  {
    if ((j.model1 == model1) && (j.model2 == model2) && (j.link1 == link1) && (j.link2 == link2))
    {
      joint = j;
      return true;
    }
  }
  return false;

}

bool GazeboRosLinkAttacher::attach_callback(gazebo_ros_link_attacher::Attach::Request &req,
                                            gazebo_ros_link_attacher::Attach::Response &res)
{
  ROS_INFO_STREAM("Received request to attach model: '" << req.model_name_1
                                                        << "' using link: '" << req.link_name_1 << "' with model: '"
                                                        << req.model_name_2 << "' using link: '" << req.link_name_2
                                                        << "'");
  std::optional<ignition::math::Vector3d> anchor_position;
  if (req.has_anchor_position)
  {
    ROS_INFO_STREAM("with anchor position");
    ROS_INFO_STREAM(req.anchor_position);
    anchor_position.emplace(req.anchor_position.x, req.anchor_position.y, req.anchor_position.z);
  }
  if (!this->attach(req.model_name_1, req.link_name_1,
                    req.model_name_2, req.link_name_2,
                    anchor_position))
  {
    ROS_ERROR_STREAM("Could not make the attach.");
    res.ok = false;
  } else
  {
    ROS_INFO_STREAM("Attach was successful");
    res.ok = true;
  }
  return true;

}

bool GazeboRosLinkAttacher::detach_callback(gazebo_ros_link_attacher::Attach::Request &req,
                                            gazebo_ros_link_attacher::Attach::Response &res)
{
  ROS_INFO_STREAM("Received request to detach model: '" << req.model_name_1
                                                        << "' using link: '" << req.link_name_1 << "' with model: '"
                                                        << req.model_name_2 << "' using link: '" << req.link_name_2
                                                        << "'");
  if (!this->detach(req.model_name_1, req.link_name_1,
                    req.model_name_2, req.link_name_2))
  {
    ROS_ERROR_STREAM("Could not make the detach.");
    res.ok = false;
  } else
  {
    ROS_INFO_STREAM("Detach was successful");
    res.ok = true;
  }
  return true;
}

}
