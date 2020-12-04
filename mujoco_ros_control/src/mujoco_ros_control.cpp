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
* @file   mujoco_ros_control.cpp
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Hardware interface for simulated robot in Mujoco
**/


#include <boost/bind.hpp>
#include <mujoco_ros_control/mujoco_ros_control.h>
#include <mujoco_ros_control/visualization_utils.h>
#include <urdf/model.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>

namespace mujoco_ros_control
{
MujocoRosControl::MujocoRosControl()
: n_free_joints_(0)
{
}

MujocoRosControl::~MujocoRosControl()
{
  // deallocate existing mjModel
  mj_deleteModel(mujoco_model);

  // deallocate existing mjData
  mj_deleteData(mujoco_data);
  mj_deactivate();
}

bool MujocoRosControl::init(ros::NodeHandle &nodehandle)
{
      // Check that ROS has been initialized
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("mujoco_ros_control", "Unable to initialize Mujoco node.");
        return false;
    }

    if (nodehandle.getParam("mujoco_ros_control/key_path", key_path_))
    {
      ROS_INFO("Got param activation key path: %s", key_path_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'key_path', attempting activation with default ('%s')", key_path_.c_str());
    }

    // visualize or not
    nodehandle.getParam("mujoco_ros_control/visualize", visualize_);

    // get window size
    nodehandle.getParam("mujoco_ros_control/window_width", window_width_);
    nodehandle.getParam("mujoco_ros_control/window_height", window_height_);

    // activation license mujoco
    mj_activate(key_path_.c_str());

    // publish clock for simulated time
    pub_clock_ = nodehandle.advertise<rosgraph_msgs::Clock>("/clock", 10);

    // create robot node handle
    robot_node_handle = ros::NodeHandle("/");

    ROS_INFO_NAMED("mujoco_ros_control", "Starting mujoco_ros_control node in namespace: %s", robot_namespace_.c_str());

    // read urdf from ros parameter server then setup actuators and mechanism control node.
    if (nodehandle.getParam("mujoco_ros_control/robot_description_param", robot_description_param_))
    {
      ROS_INFO("Got param Robot description: %s", robot_description_param_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'robot_description_param'");
    }

    const std::string urdf_string = get_urdf(robot_description_param_);

    if (!parse_transmissions(urdf_string))
    {
      ROS_ERROR_NAMED("mujoco_ros_control", "Error parsing URDF in mujoco_ros_control node, node not active.\n");
      return false;
    }

    // read xml from ros parameter server
    // const std::string xml_string = get_mujoco_xml("mujoco_model_xml");

    // std::string tmp_filename("/tmp/tmp_mujoco_xml.XXXXXX");
    // int tmp_fd = mkstemp((char *)tmp_filename.c_str());
    // if (tmp_fd == -1) {
    //   ROS_ERROR_NAMED("mujoco_ros_control", "Error creating temporary xml file\n");
    //   return false;
    // }
    // write(tmp_fd, xml_string.c_str(), xml_string.length());
    // close(tmp_fd);

    if (nodehandle.getParam("mujoco_ros_control/robot_model_path", robot_model_path_))
    {
      ROS_INFO("Got param: %s", robot_model_path_.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'robot_model_path'");
    }

    char error[1000];

    // create mjModel
    mujoco_model = mj_loadXML(robot_model_path_.c_str(), NULL, error, 1000);
    // mujoco_model = mj_loadXML(tmp_filename.c_str(), NULL, error, 1000);
    if (!mujoco_model)
    {
      printf("Could not load mujoco model with error: %s.\n", error);
      return false;
    }

    // unlink(tmp_filename.c_str());// delete temporary file

    // create mjData corresponding to mjModel
    mujoco_data = mj_makeData(mujoco_model);
    if (!mujoco_data)
    {
      printf("Could not create mujoco data from model.\n");
      return false;
    }

    // check number of dofs
    get_number_of_dofs();

    // get the Mujoco simulation period
    ros::Duration mujoco_period(mujoco_model->opt.timestep);

    // set control period as mujoco_period
    control_period_ = mujoco_period;

    // set mujoco model parameters
    set_model_parameters();

    // load the RobotHWSim abstraction to interface the controllers with the gazebo model
    try
    {
      robot_hw_sim_loader_.reset
        (new pluginlib::ClassLoader<mujoco_ros_control::RobotHWSimPlugin>
          ("mujoco_ros_control", "mujoco_ros_control::RobotHWSimPlugin"));

    robot_hw_sim_ = robot_hw_sim_loader_->createInstance("mujoco_ros_control/RobotHWSim");
    urdf::Model urdf_model;
    const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

    // get robot links from urdf
    std::map<std::string, urdf::LinkSharedPtr> robot_links;
    robot_links = urdf_model_ptr->links_;
    std::map<std::string, urdf::LinkSharedPtr>::iterator it;
    for (it = robot_links.begin(); it != robot_links.end(); ++it)
    {
      robot_link_names_.push_back(it->first);
    }

    // check for objects
    check_objects_in_scene();

    ROS_INFO("Initialising robot simulation interface...");
    try
    {
      if (!robot_hw_sim_->init_sim(robot_namespace_, robot_node_handle, mujoco_model,
                                  mujoco_data, urdf_model_ptr, transmissions_, n_free_joints_))
      {
        ROS_FATAL_NAMED("mujoco_ros_control", "Could not initialize robot sim interface");
        return false;
      }
    }
    catch (std::exception &e)
    {
      ROS_ERROR("Failed to initialise robot simulation interface.");
      ROS_ERROR("%s", e.what());
      return false;
    }

    // create the controller manager
    controller_manager_.reset
      (new controller_manager::ControllerManager(robot_hw_sim_.get(), robot_node_handle));
    }
    catch(pluginlib::LibraryLoadException &ex)
    {
      ROS_FATAL_STREAM_NAMED("mujoco_ros_control" , "Failed to create robot sim interface loader: "
                             << ex.what());
    }
    ROS_INFO_NAMED("mujoco_ros_control", "Loaded mujoco_ros_control.");

    // set up the initial simulation environment
    setup_sim_environment();
    return true;
}

void MujocoRosControl::setup_sim_environment()
{
  XmlRpc::XmlRpcValue robot_joints, robot_initial_state;
  int joint_id;
  int joint_qpos_addr;
  bool params_read_correctly = true;

  if (!robot_node_handle.getParam("robot_joints", robot_joints))
  {
    ROS_WARN("Failed to get param 'robot_joints'");
    params_read_correctly = false;
  }

  // reset simulation
  mj_resetData(mujoco_model, mujoco_data);

  if (params_read_correctly && robot_node_handle.getParam("robot_initial_state", robot_initial_state))
  {
    for (int i = 0; i < robot_joints.size(); i++)
    {
      for (XmlRpc::XmlRpcValue::iterator it = robot_initial_state.begin(); it != robot_initial_state.end(); ++it)
      {
        if (robot_joints[i] == it->first)
        {
          joint_id = mj_name2id(mujoco_model, mjOBJ_JOINT, it->first.c_str());
          joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
          mujoco_data->qpos[joint_qpos_addr] = it->second;
        }
      }
    }
  }
  else
  {
    ROS_WARN("Failed to get param 'robot_initial_state'");
    params_read_correctly = false;
  }

  if (!params_read_correctly)
  {
    for (int i = 0; i < n_dof_; i++)
    {
      if (mujoco_model->jnt_type[i] != mjJNT_FREE && mujoco_model->jnt_type[i] != mjJNT_BALL)
      {
        joint_qpos_addr = mujoco_model->jnt_qposadr[i];
        mujoco_data->qpos[joint_qpos_addr] = 0;
      }
    }
  }

  // compute forward kinematics for new pos
  mj_forward(mujoco_model, mujoco_data);

  // TODO (chongyi zheng): delete this
  // for (int i = 0; i < robot_joints.size(); i++)
  // {
  //   for (XmlRpc::XmlRpcValue::iterator it = robot_initial_state.begin(); it != robot_initial_state.end(); ++it)
  //   {
  //     if (robot_joints[i] == it->first)
  //     {
  //       joint_id = mj_name2id(mujoco_model, mjOBJ_JOINT, it->first.c_str());
  //       joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
  //       ROS_INFO_STREAM(robot_joints[i] << ": " << mujoco_data->qpos[joint_qpos_addr]);
  //     }
  //   }
  // }

  // run simulation to setup the new pos
  mj_step(mujoco_model, mujoco_data);

  // TODO (chongyi zheng): delete this
  // for (int i = 0; i < robot_joints.size(); i++)
  // {
  //   for (XmlRpc::XmlRpcValue::iterator it = robot_initial_state.begin(); it != robot_initial_state.end(); ++it)
  //   {
  //     if (robot_joints[i] == it->first)
  //     {
  //       joint_id = mj_name2id(mujoco_model, mjOBJ_JOINT, it->first.c_str());
  //       joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
  //       ROS_INFO_STREAM(robot_joints[i] << ": " << mujoco_data->qpos[joint_qpos_addr]);
  //     }
  //   }
  // }
}

void MujocoRosControl::update()
{
  publish_sim_time();

  ros::Time sim_time = (ros::Time)mujoco_data->time;
  ros::Time sim_time_ros(sim_time.sec, sim_time.nsec);

  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  mj_step1(mujoco_model, mujoco_data);

  // check if we should update the controllers
  if (sim_period >= control_period_)
  {
    // store simulation time
    last_update_sim_time_ros_ = sim_time_ros;

    // update the robot simulation with the state of the mujoco model
    robot_hw_sim_->read(sim_time_ros, sim_period);

    bool reset_ctrls = false;

    // compute the controller commands
    controller_manager_->update(sim_time_ros, sim_period, reset_ctrls);
  }

  // update the mujoco model with the result of the controller
  robot_hw_sim_->write(sim_time_ros, sim_time_ros - last_write_sim_time_ros_);

  last_write_sim_time_ros_ = sim_time_ros;
  mj_step2(mujoco_model, mujoco_data);

  publish_objects_in_scene();
  // publish_joint_states();
  // publish_site_states();
  // publish_body_states();
}

// get the MuJoCo XML from the parameter server
std::string MujocoRosControl::get_mujoco_xml(std::string param_name) const
{
  std::string xml_string;
  while (xml_string.empty())
  {
    std::string search_param_name;
    if (robot_node_handle.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control is waiting for model"
        " XML in parameter [%s] on the ROS param server.", search_param_name.c_str());

      robot_node_handle.getParam(search_param_name, xml_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control is waiting for model"
        " XML in parameter [%s] on the ROS param server.", robot_model_xml_.c_str());

      robot_node_handle.getParam(param_name, xml_string);
    }

    usleep(100000);
  }
  ROS_INFO_STREAM_NAMED("mujoco_ros_control", "Received XML from param server, parsing...");

  return xml_string;
}

// get the URDF XML from the parameter server
std::string MujocoRosControl::get_urdf(std::string param_name) const
{
  std::string urdf_string;

  // search and wait for robot_description on param server
  while (urdf_string.empty())
  {
    std::string search_param_name;
    if (robot_node_handle.searchParam(param_name, search_param_name))
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

      robot_node_handle.getParam(search_param_name, urdf_string);
    }
    else
    {
      ROS_INFO_ONCE_NAMED("mujoco_ros_control", "mujoco_ros_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.", robot_description_param_.c_str());

      robot_node_handle.getParam(param_name, urdf_string);
    }

    usleep(100000);
  }
  ROS_DEBUG_STREAM_NAMED("mujoco_ros_control", "Received urdf from param server, parsing...");

  return urdf_string;
}

// get Transmissions from the URDF
bool MujocoRosControl::parse_transmissions(const std::string& urdf_string)
{
  transmission_interface::TransmissionParser::parse(urdf_string, transmissions_);
  return true;
}

std::string MujocoRosControl::geom_type_to_string(int geom_type)
{
  std::string result;
  switch (geom_type)
  {
    case 0 :
      result = mujoco_ros_msgs::ModelStates::PLANE;
      break;
    case 1 :
      result = mujoco_ros_msgs::ModelStates::HFIELD;
      break;
    case 2 :
      result = mujoco_ros_msgs::ModelStates::SPHERE;
      break;
    case 3 :
      result = mujoco_ros_msgs::ModelStates::CAPSULE;
      break;
    case 4 :
      result = mujoco_ros_msgs::ModelStates::ELLIPSOID;
      break;
    case 5 :
      result = mujoco_ros_msgs::ModelStates::CYLINDER;
      break;
    case 6 :
      result = mujoco_ros_msgs::ModelStates::BOX;
      break;
    case 7 :
      result = mujoco_ros_msgs::ModelStates::MESH;
      break;
    default:
      result = "unknown_type";
      break;
  }
  return result;
}

void MujocoRosControl::get_number_of_dofs()
{
  n_dof_ = mujoco_model->njnt;
}

std::vector<XmlRpc::XmlRpcValue> MujocoRosControl::extract_name_index_parameters(int *name_adr, int n, mjtObj obj_type)
{
  int id;
  std::string name;
  XmlRpc::XmlRpcValue name2id;
  XmlRpc::XmlRpcValue id2name;

  for (int i = 0; i < n; i++)
  {
    name = mujoco_model->names + name_adr[i];
    if (name.length() > 0)
    {
      id = mj_name2id(mujoco_model, obj_type, name.c_str());
      name2id[name] = id;
      id2name[id] = name;
    }
  }

  return {name2id, id2name};
}

void MujocoRosControl::set_model_parameters()
{
  // timestep
  robot_node_handle.setParam("mujoco_ros/timestep", mujoco_model->opt.timestep);

  // ncam
  robot_node_handle.setParam("mujoco_ros/ncam", mujoco_model->ncam);

  // joint_pos_indexes, joint_vel_indexes
  XmlRpc::XmlRpcValue joint_pos_indexes, joint_vel_indexes;
  std::string joint_name;
  int joint_type, pos_ndim, vel_ndim, joint_qpos_addr, joint_qvel_addr;

  for (int joint_id = 0; joint_id < n_dof_; joint_id++)
  {
    joint_name = mj_id2name(mujoco_model, mjOBJ_JOINT, joint_id);
    joint_type = mujoco_model->jnt_type[joint_id];
    joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
    joint_qvel_addr = mujoco_model->jnt_dofadr[joint_id];
    switch (joint_type)
    {
      case mjJNT_FREE:
        pos_ndim = 7;
        vel_ndim = 6;
        break;
      case mjJNT_BALL:
        pos_ndim = 4;
        vel_ndim = 3;
        break;
      default:  // mjJNT_HINGE, mjJNT_SLIDE
        pos_ndim = 1;
        vel_ndim = 1;
        break;
    }

    if (pos_ndim == 1)
    {
      joint_pos_indexes[joint_name] = joint_qpos_addr;
    }
    else
    {
      // joint_pos_index_ranges[joint_name] = {joint_qpos_addr, joint_qpos_addr + pos_ndim};
      XmlRpc::XmlRpcValue joint_qpos_addr_vec;
      joint_qpos_addr_vec[0] = joint_qpos_addr;
      joint_qpos_addr_vec[1] = joint_qpos_addr + pos_ndim;
      joint_pos_indexes[joint_name] = joint_qpos_addr_vec;
    }
    

    if (vel_ndim == 1)
    {
      joint_vel_indexes[joint_name] = joint_qvel_addr;
    }
    else
    {
      // joint_vel_index_ranges[joint_name] = {joint_qvel_addr, joint_qvel_addr + vel_ndim};
      XmlRpc::XmlRpcValue joint_qvel_addr_vec;
      joint_qvel_addr_vec[0] = joint_qvel_addr;
      joint_qvel_addr_vec[1] = joint_qvel_addr + vel_ndim;
      joint_vel_indexes[joint_name] = joint_qvel_addr_vec;
    }
  }
  
  robot_node_handle.setParam("mujoco_ros/joint_pos_indexes", joint_pos_indexes);
  robot_node_handle.setParam("mujoco_ros/joint_vel_indexes", joint_vel_indexes);

  // self.body_names, self._body_name2id, self._body_id2name = self._extract_mj_names(p, p.name_bodyadr, p.nbody, mjtObj.mjOBJ_BODY)
  // self.joint_names, self._joint_name2id, self._joint_id2name = self._extract_mj_names(p, p.name_jntadr, p.njnt, mjtObj.mjOBJ_JOINT)
  // self.geom_names, self._geom_name2id, self._geom_id2name = self._extract_mj_names(p, p.name_geomadr, p.ngeom, mjtObj.mjOBJ_GEOM)
  // self.site_names, self._site_name2id, self._site_id2name = self._extract_mj_names(p, p.name_siteadr, p.nsite, mjtObj.mjOBJ_SITE)
  // self.light_names, self._light_name2id, self._light_id2name = self._extract_mj_names(p, p.name_lightadr, p.nlight, mjtObj.mjOBJ_LIGHT)
  // self.camera_names, self._camera_name2id, self._camera_id2name = self._extract_mj_names(p, p.name_camadr, p.ncam, mjtObj.mjOBJ_CAMERA)
  // self.actuator_names, self._actuator_name2id, self._actuator_id2name = self._extract_mj_names(p, p.name_actuatoradr, p.nu, mjtObj.mjOBJ_ACTUATOR)
  // self.sensor_names, self._sensor_name2id, self._sensor_id2name = self._extract_mj_names(p, p.name_sensoradr, p.nsensor, mjtObj.mjOBJ_SENSOR)
  // self.tendon_names, self._tendon_name2id, self._tendon_id2name = self._extract_mj_names(p, p.name_tendonadr, p.ntendon, mjtObj.mjOBJ_TENDON)
  // self.mesh_names, self._mesh_name2id, self._mesh_id2name = self._extract_mj_names(p, p.name_meshadr, p.nmesh, mjtObj.mjOBJ_MESH)

  // body_name2id, body_id2name
  std::vector<XmlRpc::XmlRpcValue> body_params;
  body_params = extract_name_index_parameters(mujoco_model->name_bodyadr, mujoco_model->nbody, mjOBJ_BODY);

  robot_node_handle.setParam("mujoco_ros/body_name2id", body_params[0]);
  robot_node_handle.setParam("mujoco_ros/body_id2name", body_params[1]);

  // joint_name2id, joint_id2name
  std::vector<XmlRpc::XmlRpcValue> joint_params;
  joint_params = extract_name_index_parameters(mujoco_model->name_jntadr, mujoco_model->njnt, mjOBJ_JOINT);

  robot_node_handle.setParam("mujoco_ros/joint_name2id", joint_params[0]);
  robot_node_handle.setParam("mujoco_ros/joint_id2name", joint_params[1]);

  // geom_name2id, geom_id2name
  std::vector<XmlRpc::XmlRpcValue> geom_params;
  geom_params = extract_name_index_parameters(mujoco_model->name_geomadr, mujoco_model->ngeom, mjOBJ_GEOM);

  robot_node_handle.setParam("mujoco_ros/geom_name2id", geom_params[0]);
  robot_node_handle.setParam("mujoco_ros/geom_id2name", geom_params[1]);

  // site_name2id, site_id2name
  std::vector<XmlRpc::XmlRpcValue> site_params;
  site_params = extract_name_index_parameters(mujoco_model->name_siteadr, mujoco_model->nsite, mjOBJ_SITE);

  robot_node_handle.setParam("mujoco_ros/site_name2id", site_params[0]);
  robot_node_handle.setParam("mujoco_ros/site_id2name", site_params[1]);

  // camera_name2id, camera_id2name
  std::vector<XmlRpc::XmlRpcValue> camera_params;
  camera_params = extract_name_index_parameters(mujoco_model->name_camadr, mujoco_model->ncam, mjOBJ_CAMERA);

  robot_node_handle.setParam("mujoco_ros/camera_name2id", camera_params[0]);
  robot_node_handle.setParam("mujoco_ros/camera_id2name", camera_params[1]);

  // actuator_name2id, actuator_id2name
  std::vector<XmlRpc::XmlRpcValue> actuator_params;
  actuator_params = extract_name_index_parameters(mujoco_model->name_actuatoradr, mujoco_model->nu, mjOBJ_ACTUATOR);

  robot_node_handle.setParam("mujoco_ros/actuator_name2id", actuator_params[0]);
  robot_node_handle.setParam("mujoco_ros/actuator_id2name", actuator_params[1]);

  // actuator_ctrlrange
  // XmlRpc::XmlRpcValue actuator_ctrlrange(mujoco_model->actuator_ctrlrange);
  // self._actuator_ctrlrange = _wrap_mjtNum_2d(p.actuator_ctrlrange, p.nu, 2)
  // cdef inline np.ndarray _wrap_mjtNum_2d(mjtNum* a, int shape0, int shape1):
  //   if shape0 * shape1 == 0: return None
  //   cdef mjtNum[:,:] b = <mjtNum[:shape0,:shape1]> a
  //   return np.asarray(b)
  
  // mujoco_model->actuator_ctrlrange

}

void MujocoRosControl::publish_sim_time()
{
  ros::Time sim_time = (ros::Time)mujoco_data->time;
  if (pub_clock_frequency_ > 0 && (sim_time - last_pub_clock_time_).toSec() < 1.0/pub_clock_frequency_)
    return;

  ros::Time current_time = (ros::Time)mujoco_data->time;
  rosgraph_msgs::Clock ros_time_;
  ros_time_.clock.fromSec(current_time.toSec());
  // publish time to ros
  last_pub_clock_time_ = sim_time;
  pub_clock_.publish(ros_time_);
}

void MujocoRosControl::check_objects_in_scene()
{
  int num_of_bodies = mujoco_model->nbody;
  int object_id;
  int joint_addr;
  int joint_type;
  int num_of_joints_for_body;
  std::string object_name;

  for (int object_id=0; object_id < num_of_bodies; object_id++)
  {
    object_name = mj_id2name(mujoco_model, 1, object_id);
    num_of_joints_for_body = mujoco_model->body_jntnum[object_id];
    if (0 == num_of_joints_for_body &&
        !(std::find(robot_link_names_.begin(), robot_link_names_.end(), object_name) != robot_link_names_.end()))
    {
      objects_in_scene_[object_id] = STATIC;
      ROS_INFO_STREAM("Static object found: " << object_name);
    }
    else if (1 == num_of_joints_for_body)
    {
      joint_addr = mujoco_model->body_jntadr[object_id];
      joint_type = mujoco_model->jnt_type[joint_addr];
      if (0 == joint_type)
      {
        objects_in_scene_[object_id] = FREE;
        n_free_joints_++;
        ROS_INFO_STREAM("Free object found: " << object_name);
      }
    }
  }
}

void MujocoRosControl::publish_objects_in_scene()
{
  const int geom_size_dim = 3;
  const int xpos_dim = 3;
  const int xquat_dim = 4;
  int geom_type;
  int geom_addr;
  geometry_msgs::Pose pose;
  std_msgs::Float64MultiArray size;
  mujoco_ros_msgs::ModelStates objects;

  for (std::map<int, Object_State>::iterator it = objects_in_scene_.begin(); it != objects_in_scene_.end(); it++ )
  {
    size.data.clear();
    geom_addr = mujoco_model->body_geomadr[it->first];
    geom_type = mujoco_model->geom_type[geom_addr];

    for (int i=0; i < geom_size_dim; i++)
    {
      size.data.push_back(mujoco_model->geom_size[3 * geom_addr + i]);
    }

    pose.position.x = mujoco_data->xpos[xpos_dim * it->first];
    pose.position.y = mujoco_data->xpos[xpos_dim * it->first + 1];
    pose.position.z = mujoco_data->xpos[xpos_dim * it->first + 2];
    pose.orientation.x = mujoco_data->xquat[xquat_dim * it->first + 1];
    pose.orientation.y = mujoco_data->xquat[xquat_dim * it->first + 2];
    pose.orientation.z = mujoco_data->xquat[xquat_dim * it->first + 3];
    pose.orientation.w = mujoco_data->xquat[xquat_dim * it->first];

    objects.name.push_back(mj_id2name(mujoco_model, 1, it->first));
    objects.type.push_back(geom_type_to_string(geom_type));
    objects.is_static.push_back(it->second);
    objects.size.push_back(size);
    objects.pose.push_back(pose);
  }

  objects_in_scene_publisher.publish(objects);
}

void MujocoRosControl::publish_joint_states()
{
  mujoco_ros_msgs::JointStates joint_states;
  std::string joint_name;
  std_msgs::Float64MultiArray position, velocity;
  int joint_type, pos_ndim, vel_ndim, joint_qpos_addr, joint_qvel_addr;

  for (int joint_id = 0; joint_id < n_dof_; joint_id++)
  {
    joint_name = mj_id2name(mujoco_model, mjOBJ_JOINT, joint_id);
    joint_type = mujoco_model->jnt_type[joint_id];
    joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
    joint_qvel_addr = mujoco_model->jnt_dofadr[joint_id];
    switch (joint_type)
    {
      case mjJNT_FREE:
        pos_ndim = 7;
        vel_ndim = 6;
        break;
      case mjJNT_BALL:
        pos_ndim = 4;
        vel_ndim = 3;
        break;
      default:  // mjJNT_HINGE, mjJNT_SLIDE
        pos_ndim = 1;
        vel_ndim = 1;
        break;
    }

    joint_states.name.push_back(joint_name);
    position.data.clear();
    velocity.data.clear();
    if (pos_ndim == 1)
    {
      position.data.push_back(mujoco_data->qpos[joint_qpos_addr]);
    }
    else
    {
      for (int idx = 0; idx < pos_ndim; idx++)
      {
        position.data.push_back(mujoco_data->qpos[joint_qpos_addr + idx]);
      }
    }

    if (vel_ndim == 1)
    {
      velocity.data.push_back(mujoco_data->qvel[joint_qvel_addr]);
    }
    else
    {
      for (int idx = 0; idx < vel_ndim; idx++)
      {
        velocity.data.push_back(mujoco_data->qvel[joint_qvel_addr + idx]);
      }
    }
    joint_states.position.push_back(position);
    joint_states.velocity.push_back(velocity);
  }
  
  joint_state_publisher.publish(joint_states);
}

void MujocoRosControl::publish_site_states()
{
  mujoco_ros_msgs::SiteStates site_states;
  std::string site_name;
  std_msgs::Float64MultiArray position, rotation_matrix;

  for (int site_id = 0; site_id < mujoco_model->nsite; site_id++)
  {
    site_name = mj_id2name(mujoco_model, mjOBJ_SITE, site_id);

    site_states.name.push_back(site_name);
    position.data.clear();
    rotation_matrix.data.clear();
    // reference: http://www.mujoco.org/forum/index.php?threads/cartesian-coordinates-of-body.3376/
    for (int idx = 0; idx < 3; idx++)
    {
      position.data.push_back(mujoco_data->site_xpos[3 * site_id + idx]);
    }

    for (int idx = 0; idx < 9; idx++)
    {
      rotation_matrix.data.push_back(mujoco_data->site_xmat[9 * site_id + idx]);// row first layout
    }
    site_states.position.push_back(position);
    site_states.rotation_matrix.push_back(rotation_matrix);
  }
  
  site_state_publisher.publish(site_states);
}

void MujocoRosControl::publish_body_states()
{
  mujoco_ros_msgs::BodyStates body_states;
  std::string body_name;
  geometry_msgs::Pose pose;

  for (int body_id = 0; body_id < mujoco_model->nbody; body_id++)
  {
    body_name = mj_id2name(mujoco_model, mjOBJ_BODY, body_id);
    body_states.name.push_back(body_name);
    
    // reference: http://www.mujoco.org/forum/index.php?threads/cartesian-coordinates-of-body.3376/
    pose.position.x = mujoco_data->xpos[3 * body_id];
    pose.position.y = mujoco_data->xpos[3 * body_id + 1];
    pose.position.z = mujoco_data->xpos[3 * body_id + 2];
    pose.orientation.w = mujoco_data->xquat[4 * body_id];// mujoco xquat format = wxyz
    pose.orientation.x = mujoco_data->xquat[4 * body_id + 1];
    pose.orientation.y = mujoco_data->xquat[4 * body_id + 2];
    pose.orientation.z = mujoco_data->xquat[4 * body_id + 3];
    body_states.pose.push_back(pose);
  }

  body_state_publisher.publish(body_states);
}

// void MujocoRosControl::set_objects_in_scene_callback(const mujoco_ros_msgs::ModelStates& model_states_msg)
// {
//   int object_id;
//   int joint_id;
//   int joint_qpos_addr;

//   for (int i = 0; i < model_states_msg.name.size(); i++)
//   {
//     object_id = mj_name2id(mujoco_model, mjOBJ_BODY, model_states_msg.name[i].c_str());
//     joint_id = mujoco_model->body_jntadr[object_id];
//     joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
//     if (mujoco_model->jnt_type[joint_id] == mjJNT_FREE)
//     {
//       // position
//       mujoco_data->qpos[joint_qpos_addr] = model_states_msg.pose[i].position.x;
//       mujoco_data->qpos[joint_qpos_addr + 1] = model_states_msg.pose[i].position.y;
//       mujoco_data->qpos[joint_qpos_addr + 2] = model_states_msg.pose[i].position.z;

//       // orientation
//       mujoco_data->qpos[joint_qpos_addr + 3] = model_states_msg.pose[i].orientation.w;
//       mujoco_data->qpos[joint_qpos_addr + 4] = model_states_msg.pose[i].orientation.x;
//       mujoco_data->qpos[joint_qpos_addr + 5] = model_states_msg.pose[i].orientation.y;
//       mujoco_data->qpos[joint_qpos_addr + 6] = model_states_msg.pose[i].orientation.z;
//     }
//     else
//     {
//       ROS_WARN("Only objects with free joints can be set!");
//     }
//   }
// }

bool MujocoRosControl::set_joint_qpos_callback(mujoco_ros_msgs::SetJointQPos::Request& req, mujoco_ros_msgs::SetJointQPos::Response& res)
{
  std::vector<std::string> names = req.name;
  std::vector<std_msgs::Float64MultiArray> values = req.value;

  int joint_id, joint_type, joint_qpos_addr, pos_ndim;
  std::vector<double> value;
  for (int i = 0; i < names.size(); i++)
  {
    joint_id = mj_name2id(mujoco_model, mjOBJ_JOINT, names[i].c_str());
    joint_type = mujoco_model->jnt_type[joint_id];
    joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
    switch (joint_type)
    {
      case mjJNT_FREE:
        pos_ndim = 7;
        break;
      case mjJNT_BALL:
        pos_ndim = 4;
        break;
      default:  // mjJNT_HINGE, mjJNT_SLIDE
        pos_ndim = 1;
        break;
    }

    value = values[i].data;

    for (int j = 0; j < pos_ndim; j++)
    {
      mujoco_data->qpos[joint_qpos_addr + j] = value[j];
    }
  }

  return true;
}

bool MujocoRosControl::set_ctrl_callback(mujoco_ros_msgs::SetCtrl::Request& req, mujoco_ros_msgs::SetCtrl::Response& res)
{
  std::vector<std::string> names = req.name;
  std::vector<double> ctrls = req.ctrl;
  int actuator_id;

  for (int i = 0; i < names.size(); i++)
  {
    ROS_INFO_STREAM("actuator_name: " << names[i]);

    actuator_id = mj_name2id(mujoco_model, mjOBJ_ACTUATOR, names[i].c_str());
    ROS_INFO_STREAM("actuator_id: " << actuator_id);
    ROS_INFO_STREAM("init_actuator_value: " << mujoco_data->ctrl[actuator_id]);
    mujoco_data->ctrl[actuator_id] = ctrls[i];
    ROS_INFO_STREAM("actuator_value: " << mujoco_data->ctrl[actuator_id]);
  }

  return true;
}

bool MujocoRosControl::set_vopt_geomgroup_callback(mujoco_ros_msgs::SetOptGeomGroup::Request& req, mujoco_ros_msgs::SetOptGeomGroup::Response& res)
{
  int32_t index = req.index;
  uint8_t value = req.value;

  visualization_utils->get_opt()->geomgroup[index] = value;

  return true;
}

bool MujocoRosControl::reset_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  XmlRpc::XmlRpcValue robot_initial_state;
  int joint_id;
  int joint_qpos_addr;

  if (robot_node_handle.getParam("robot_initial_state", robot_initial_state))
  {
    for (XmlRpc::XmlRpcValue::iterator it = robot_initial_state.begin(); it != robot_initial_state.end(); ++it)
    {
      joint_id = mj_name2id(mujoco_model, mjOBJ_JOINT, it->first.c_str());
      joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
      mujoco_data->qpos[joint_qpos_addr] = it->second;

      res.message = "Reset successed";
      res.success = true;
    }
  }
  else
  {
    ROS_WARN("Failed to reset from param 'robot_initial_state'");
    res.message = "Failed to reset from param 'robot_initial_state'";
    res.success = false;
  }

  return true;
}

bool MujocoRosControl::set_fixed_camera_callback(mujoco_ros_msgs::SetFixedCamera::Request& req, mujoco_ros_msgs::SetFixedCamera::Response& res)
{
  int32_t camera_id = req.camera_id;
  
  visualization_utils->get_cam()->fixedcamid = camera_id;
  visualization_utils->get_cam()->type = mjCAMERA_FIXED;

  return true;
}

bool MujocoRosControl::get_body_states_callback(mujoco_ros_msgs::GetBodyStates::Request& req, mujoco_ros_msgs::GetBodyStates::Response& res)
{
  std::string body_name;
  geometry_msgs::Pose pose;

  for (int body_id = 0; body_id < mujoco_model->nbody; body_id++)
  {
    body_name = mj_id2name(mujoco_model, mjOBJ_BODY, body_id);
    res.name.push_back(body_name);
    
    // reference: http://www.mujoco.org/forum/index.php?threads/cartesian-coordinates-of-body.3376/
    pose.position.x = mujoco_data->xpos[3 * body_id];
    pose.position.y = mujoco_data->xpos[3 * body_id + 1];
    pose.position.z = mujoco_data->xpos[3 * body_id + 2];
    pose.orientation.w = mujoco_data->xquat[4 * body_id];// mujoco xquat format = wxyz
    pose.orientation.x = mujoco_data->xquat[4 * body_id + 1];
    pose.orientation.y = mujoco_data->xquat[4 * body_id + 2];
    pose.orientation.z = mujoco_data->xquat[4 * body_id + 3];
    res.pose.push_back(pose);
  }

  return true;
}

bool MujocoRosControl::get_joint_states_callback(mujoco_ros_msgs::GetJointStates::Request& req, mujoco_ros_msgs::GetJointStates::Response& res)
{
  std::string joint_name;
  std_msgs::Float64MultiArray position, velocity;
  int joint_type, pos_ndim, vel_ndim, joint_qpos_addr, joint_qvel_addr;

  for (int joint_id = 0; joint_id < n_dof_; joint_id++)
  {
    joint_name = mj_id2name(mujoco_model, mjOBJ_JOINT, joint_id);
    joint_type = mujoco_model->jnt_type[joint_id];
    joint_qpos_addr = mujoco_model->jnt_qposadr[joint_id];
    joint_qvel_addr = mujoco_model->jnt_dofadr[joint_id];
    switch (joint_type)
    {
      case mjJNT_FREE:
        pos_ndim = 7;
        vel_ndim = 6;
        break;
      case mjJNT_BALL:
        pos_ndim = 4;
        vel_ndim = 3;
        break;
      default:  // mjJNT_HINGE, mjJNT_SLIDE
        pos_ndim = 1;
        vel_ndim = 1;
        break;
    }

    res.name.push_back(joint_name);
    position.data.clear();
    velocity.data.clear();
    if (pos_ndim == 1)
    {
      position.data.push_back(mujoco_data->qpos[joint_qpos_addr]);
    }
    else
    {
      for (int idx = 0; idx < pos_ndim; idx++)
      {
        position.data.push_back(mujoco_data->qpos[joint_qpos_addr + idx]);
      }
    }

    if (vel_ndim == 1)
    {
      velocity.data.push_back(mujoco_data->qvel[joint_qvel_addr]);
    }
    else
    {
      for (int idx = 0; idx < vel_ndim; idx++)
      {
        velocity.data.push_back(mujoco_data->qvel[joint_qvel_addr + idx]);
      }
    }
    res.position.push_back(position);
    res.velocity.push_back(velocity);
  }
  
  return true;
}

bool MujocoRosControl::get_site_states_callback(mujoco_ros_msgs::GetSiteStates::Request& req, mujoco_ros_msgs::GetSiteStates::Response& res)
{
  std::string site_name;
  std_msgs::Float64MultiArray position, rotation_matrix;

  for (int site_id = 0; site_id < mujoco_model->nsite; site_id++)
  {
    site_name = mj_id2name(mujoco_model, mjOBJ_SITE, site_id);

    res.name.push_back(site_name);
    position.data.clear();
    rotation_matrix.data.clear();
    // reference: http://www.mujoco.org/forum/index.php?threads/cartesian-coordinates-of-body.3376/
    for (int idx = 0; idx < 3; idx++)
    {
      position.data.push_back(mujoco_data->site_xpos[3 * site_id + idx]);
    }

    for (int idx = 0; idx < 9; idx++)
    {
      rotation_matrix.data.push_back(mujoco_data->site_xmat[9 * site_id + idx]);// row first layout
    }
    res.position.push_back(position);
    res.rotation_matrix.push_back(rotation_matrix);
  }
  
  return true;
}

}  // namespace mujoco_ros_control

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mujoco_ros_control");

    ros::NodeHandle nh_;

    mujoco_ros_control::MujocoRosControl mujoco_ros_control;

    // initialize mujoco stuff
    if (!mujoco_ros_control.init(nh_))
    {
      ROS_ERROR("Could not initialise mujoco.");
      return 1;
    }

    // spin
    ros::AsyncSpinner spinner(1);
    spinner.start();

    if (mujoco_ros_control.visualize_)
    {
      mujoco_ros_control.visualization_utils =
          &mujoco_ros_control::MujocoVisualizationUtils::getInstance();

      // init GLFW
      if ( !glfwInit() )
        mju_error("Could not initialize GLFW");

      // create window, make OpenGL context current, request v-sync
      GLFWwindow* window = glfwCreateWindow(
        mujoco_ros_control.window_width_, mujoco_ros_control.window_height_, "MujocoROS", NULL, NULL);
      glfwMakeContextCurrent(window);
      glfwSwapInterval(1);

      // make context current
      glfwMakeContextCurrent(window);

      // initialize mujoco visualization functions
      mujoco_ros_control.visualization_utils->init(mujoco_ros_control.mujoco_model, mujoco_ros_control.mujoco_data, window);

      // run main loop, target real-time simulation and 60 fps rendering
      while ( ros::ok() && !glfwWindowShouldClose(window) )
      {
        // advance interactive simulation for 1/60 sec
        // Assuming MuJoCo can simulate faster than real-time, which it usually can,
        // this loop will finish on time for the next frame to be rendered at 60 fps.
        mjtNum sim_start = mujoco_ros_control.mujoco_data->time;

        while ( mujoco_ros_control.mujoco_data->time - sim_start < 1.0/10.0 && ros::ok() ) // change the fps rate if controller become slow
        {
          mujoco_ros_control.update();
        }
        mujoco_ros_control.visualization_utils->update(window);
      }

      mujoco_ros_control.visualization_utils->terminate();
    }
    else
    {
      // run main loop, target real-time simulation and 60 fps rendering
      while ( ros::ok())
      {
        mujoco_ros_control.update();
      }
    }

    return 0;
}
