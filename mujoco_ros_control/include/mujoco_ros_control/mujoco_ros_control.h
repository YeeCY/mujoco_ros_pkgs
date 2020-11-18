/*
* Copyright 2018 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @file   mujoco_ros_control.h
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Node to allow ros_control hardware interfaces to be plugged into mujoco
**/

#ifndef MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H
#define MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>
#include <ros/package.h>

// Mujoco dependencies
#include <mujoco.h>
#include <mjdata.h>
#include <mjmodel.h>

#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <map>

// ros_control
#include <mujoco_ros_control/robot_hw_sim.h>
#include <mujoco_ros_control/robot_hw_sim_plugin.h>

// msgs
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64MultiArray.h"
#include "mujoco_ros_msgs/ModelStates.h"
#include "mujoco_ros_msgs/JointStates.h"
#include "mujoco_ros_msgs/SiteStates.h"
#include "mujoco_ros_msgs/BodyStates.h"

#include <controller_manager/controller_manager.h>
#include <transmission_interface/transmission_parser.h>

// srvs
#include "mujoco_ros_msgs/SetJointQPos.h"
#include "mujoco_ros_msgs/SetOptGeomGroup.h"
#include "mujoco_ros_msgs/SetFixedCamera.h"
#include "mujoco_ros_msgs/Reset.h"

// openGL stuff
#include <glfw3.h>
#include <mujoco_ros_control/visualization_utils.h>

#include <rosgraph_msgs/Clock.h>

namespace mujoco_ros_control
{

class MujocoRosControl
{
public:
  MujocoRosControl();
  virtual ~MujocoRosControl();

  // initialize params and controller manager
  bool init(ros::NodeHandle &nodehandle);

  // step update function
  void update();

  // pointer to the mujoco model
  mjModel* mujoco_model;
  mjData* mujoco_data;

  // mujoco visualization
  mujoco_ros_control::MujocoVisualizationUtils* visualization_utils;

  // number of degrees of freedom
  unsigned int n_dof_;

  // number of free joints in simulation
  unsigned int n_free_joints_;

protected:
  // free or static object
  enum Object_State { STATIC = true, FREE = false };

  // get the MuJoCo XML from the parameter server
  std::string get_mujoco_xml(std::string param_name) const;

  // get the URDF XML from the parameter server
  std::string get_urdf(std::string param_name) const;

  // setup initial sim environment
  void setup_sim_environment();

  // parse transmissions from URDF
  bool parse_transmissions(const std::string& urdf_string);

  // get number of degrees of freedom
  void get_number_of_dofs();

  // extract name2idx and idx2name parameters
  std::vector<XmlRpc::XmlRpcValue> extract_name_index_parameters(int *name_adr, int n, mjtObj obj_type);

  // set mujoco model parameters
  void set_model_parameters();

  // publish simulation time to ros clock
  void publish_sim_time();

  // check for free joints in the mujoco model
  void check_objects_in_scene();

  // publish free objects
  void publish_objects_in_scene();

  // publish joint states
  void publish_joint_states();

  // publish site states
  void publish_site_states();

  // publish body states
  void publish_body_states();

  // set free objects
  void set_objects_in_scene_callback(const mujoco_ros_msgs::ModelStates& model_states_msg);

  // set joint qpos
  bool set_joint_qpos_callback(mujoco_ros_msgs::SetJointQPos::Request& req, 
    mujoco_ros_msgs::SetJointQPos::Response& res);

  // set vopt geomgroup
  bool set_vopt_geomgroup(mujoco_ros_msgs::SetOptGeomGroup::Request& req, 
    mujoco_ros_msgs::SetOptGeomGroup::Response& res);

  // set fixed camera
  bool set_fixed_camera(mujoco_ros_msgs::SetFixedCamera::Request& req, 
    mujoco_ros_msgs::SetFixedCamera::Response& res);

  // // reset simulation
  // bool reset(mujoco_ros_msgs::Reset::Request& req,
  //   mujoco_ros_msgs::Reset::Response& res);

  // transform type id to type name
  std::string geom_type_to_string(int geom_id);

  // node handles
  ros::NodeHandle robot_node_handle;

  // interface loader
  boost::shared_ptr<pluginlib::ClassLoader<mujoco_ros_control::RobotHWSimPlugin> > robot_hw_sim_loader_;

  // strings
  std::string robot_namespace_;
  std::string robot_description_param_;
  std::string robot_model_path_;
  std::string robot_model_xml_;
  std::string key_path_ = "/home/yee/.mujoco/mujoco200/bin/mjkey.txt";

  // vectors
  std::vector<int> mujoco_ids;
  std::vector<int>::iterator it;
  std::vector<std::string> robot_link_names_;
  std::map<int, Object_State> objects_in_scene_;

  // transmissions in this plugin's scope
  std::vector<transmission_interface::TransmissionInfo> transmissions_;

  // robot simulator interface
  boost::shared_ptr<mujoco_ros_control::RobotHWSimPlugin> robot_hw_sim_;

  // controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // simulated clock
  ros::Publisher pub_clock_;
  int pub_clock_frequency_;
  ros::Time last_pub_clock_time_;

  // timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

  // publishing
  ros::Publisher objects_in_scene_publisher = robot_node_handle.advertise<mujoco_ros_msgs::ModelStates>
                                                                         ("mujoco_ros/model_states", 1000);
  ros::Publisher joint_state_publisher = robot_node_handle.advertise<mujoco_ros_msgs::JointStates>
                                                                         ("mujoco_ros/joint_states", 1000);
  ros::Publisher site_state_publisher = robot_node_handle.advertise<mujoco_ros_msgs::SiteStates>
                                                                         ("mujoco_ros/site_states", 1000);
  ros::Publisher body_state_publisher = robot_node_handle.advertise<mujoco_ros_msgs::BodyStates>
                                                                         ("mujoco_ros/body_states", 1000);

  // subscribing
  // ros::Subscriber set_objects_in_scene_subscriber = robot_node_handle.subscribe("/mujoco_ros/set_model_state", 1000, MujocoRosControl::set_objects_in_scene_callback);

  // server
  ros::ServiceServer set_joint_qpos_server = robot_node_handle.advertiseService(
    "mujoco_ros/set_joint_qpos", &MujocoRosControl::set_joint_qpos_callback, this);
  ros::ServiceServer set_opt_geomgroup_server = robot_node_handle.advertiseService(
    "mujoco_ros/set_vopt_geomgroup", &MujocoRosControl::set_vopt_geomgroup, this);
  ros::ServiceServer set_fixed_camera_server = robot_node_handle.advertiseService(
    "mujoco_ros/set_fixed_camera", &MujocoRosControl::set_fixed_camera, this);
  // ros::ServiceServer reset_server = robot_node_handle.advertiseService(
  //   "mujoco_ros/reset", &MujocoRosControl::reset, this);
};
}  // namespace mujoco_ros_control
#endif  // MUJOCO_ROS_CONTROL_MUJOCO_ROS_CONTROL_H
