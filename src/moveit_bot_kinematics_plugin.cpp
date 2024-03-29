#include <class_loader/class_loader.hpp>
#include <moveit_bot_kinematics_plugin/moveit_bot_kinematics_plugin.h>

// URDF, SRDF
#include <srdfdom/model.h>
#include <urdf_model/model.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_state/conversions.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// Bot kinematics
#include "bot_kinematics/bot_kinematics.h"
#include "bot_kinematics/bot_kinematics_utils.h"

// register BotKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(moveit_bot_kinematics_plugin::MoveItBotKinematicsPlugin, kinematics::KinematicsBase)

namespace moveit_bot_kinematics_plugin
{
using kinematics::KinematicsResult;

MoveItBotKinematicsPlugin::MoveItBotKinematicsPlugin() : active_(false)
{
}

bool MoveItBotKinematicsPlugin::initialize(const std::string& robot_description, const std::string& group_name,
                                           const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                           double search_discretization)
{
  bool debug = false;

  ROS_INFO_STREAM_NAMED("bot", "MoveItBotKinematicsPlugin initializing");

  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("bot", "URDF and SRDF must be loaded for SRV kinematics "
                           "solver to work.");  // TODO: is this true?
    return false;
  }

  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if (debug)
  {
    std::cout << std::endl
              << "Joint Model Variable Names: "
                 "------------------------------------------- "
              << std::endl;
    const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
    std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << std::endl;
  }

  // Get the dimension of the planning group
  dimension_ = joint_model_group_->getVariableCount();
  ROS_INFO_STREAM_NAMED("bot", "Dimension planning group '"
                                   << group_name << "': " << dimension_
                                   << ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
                                   << ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());

  // Copy joint names
  for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    ik_group_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
  }

  if (debug)
  {
    ROS_ERROR_STREAM_NAMED("bot", "tip links available:");
    std::copy(tip_frames_.begin(), tip_frames_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
  }

  // Make sure all the tip links are in the link_names vector
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    if (!joint_model_group_->hasLinkModel(tip_frames_[i]))
    {
      ROS_ERROR_NAMED("bot", "Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(),
                      group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frames_[i]);
  }

  // Choose what ROS service to send IK requests to
  ROS_DEBUG_STREAM_NAMED("bot", "Looking for ROS service name on rosparam server with param: "
                                    << "/kinematics_solver_service_name");
  std::string ik_service_name;
  lookupParam("kinematics_solver_service_name", ik_service_name, std::string("solve_ik"));

  // Setup the joint state groups that we need
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();

  // set dh parameters for bot model
  if (!setBotParameters())
  {
    ROS_ERROR_STREAM_NAMED("bot", "Could not load bot parameters. Check kinematics.yaml.");
    return false;
  }

  active_ = true;
  ROS_DEBUG_NAMED("bot", "ROS service-based kinematics solver initialized");
  return true;
}

bool MoveItBotKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  if (num_possible_redundant_joints_ < 0)
  {
    ROS_ERROR_NAMED("srv", "This group cannot have redundant joints");
    return false;
  }
  if (redundant_joints.size() > num_possible_redundant_joints_)
  {
    ROS_ERROR_NAMED("srv", "This group can only have %d redundant joints", num_possible_redundant_joints_);
    return false;
  }

  redundant_joint_indices_ = redundant_joints;

  return true;
}

bool MoveItBotKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
    if (redundant_joint_indices_[j] == index)
      return true;
  return false;
}

int MoveItBotKinematicsPlugin::getJointIndex(const std::string& name) const
{
  for (unsigned int i = 0; i < ik_group_info_.joint_names.size(); i++)
  {
    if (ik_group_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

bool MoveItBotKinematicsPlugin::timedOut(const ros::WallTime& start_time, double duration) const
{
  return ((ros::WallTime::now() - start_time).toSec() >= duration);
}

bool MoveItBotKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                              moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool MoveItBotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItBotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItBotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItBotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItBotKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const std::vector<double>& consistency_limits,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

// struct for storing and sorting solutions
struct LimitObeyingSol
{
  std::vector<double> value;
  double dist_from_seed;

  bool operator<(const LimitObeyingSol& a) const
  {
    return dist_from_seed < a.dist_from_seed;
  }
};

bool MoveItBotKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  // Check if active
  if (!active_)
  {
    ROS_ERROR_NAMED("bot", "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check if seed state correct
  if (ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("bot",
                           "Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    ROS_ERROR_STREAM_NAMED("bot", "Mismatched number of pose requests (" << ik_poses.size() << ") to tip frames ("
                                                                         << tip_frames_.size()
                                                                         << ") in searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  Eigen::Affine3d pose;
  tf::poseMsgToEigen(ik_poses[0], pose);
  std::vector<std::vector<double>> solutions;
  if (!getAllIK(pose, solutions))
  {
    ROS_INFO_STREAM_NAMED("bot", "Failed to find IK solution");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  std::vector<LimitObeyingSol> limit_obeying_solutions;

  for (auto& sol : solutions) {
      robot_state_->setJointGroupPositions(joint_model_group_, sol);
      //robot_state_->update(); // not required for checking bounds
      if (!robot_state_->satisfiesBounds(joint_model_group_)) {
          ROS_DEBUG_STREAM_NAMED("bot", "Solution is outside bounds");
          continue;
      }
      limit_obeying_solutions.push_back({sol, distance(sol, ik_seed_state)});
  }

  if (limit_obeying_solutions.empty()) {
      ROS_INFO_NAMED("bot", "None of the solutions is within joint limits");
      return false;
  }

  ROS_DEBUG_STREAM_NAMED("bot", "Solutions within limits: " << limit_obeying_solutions.size());

  //sort solutions by distance to seed state
  std::sort(limit_obeying_solutions.begin(), limit_obeying_solutions.end());

  if (!solution_callback) {
      solution = limit_obeying_solutions.front().value;
      return true;
  }

  for (auto& sol : limit_obeying_solutions) {
      solution_callback(ik_poses[0], sol.value, error_code);
      if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        solution = sol.value;
        ROS_DEBUG_STREAM_NAMED("bot", "Solution passes callback");
        return true;
      }
  }

  ROS_INFO_STREAM_NAMED("bot", "No solution fullfilled requirements of solution callback");
  return false;
}

bool MoveItBotKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                              const std::vector<double>& ik_seed_state,
                                              std::vector<std::vector<double>>& solutions, KinematicsResult& result,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  if (ik_poses.size() > 1 || ik_poses.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED("bot", "You can only get all solutions for a single pose.");
    return false;
  }
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(ik_poses[0], pose);
  return getAllIK(pose, solutions);
}

bool MoveItBotKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                              const std::vector<double>& joint_angles,
                                              std::vector<geometry_msgs::Pose>& poses) const
{
  if (!active_)
  {
    ROS_ERROR_NAMED("bot", "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    ROS_ERROR_NAMED("bot", "Joint angles vector must have size: %d", dimension_);
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != poses.size())
  {
    ROS_ERROR_STREAM_NAMED("bot", "Mismatched number of pose requests (" << poses.size() << ") to tip frames ("
                                                                         << tip_frames_.size()
                                                                         << ") in searchPositionFK");
    return false;
  }

  // forward function expect pointer to first element of array of joint values
  // that is why &joint_angles[0] is passed
  tf::poseEigenToMsg(bot_kinematics::forward(bot_parameters_, &joint_angles[0]), poses[0]);

  return true;
}

const std::vector<std::string>& MoveItBotKinematicsPlugin::getJointNames() const
{
  return ik_group_info_.joint_names;
}

const std::vector<std::string>& MoveItBotKinematicsPlugin::getLinkNames() const
{
  return ik_group_info_.link_names;
}

const std::vector<std::string>& MoveItBotKinematicsPlugin::getVariableNames() const
{
  return joint_model_group_->getVariableNames();
}

bool MoveItBotKinematicsPlugin::setBotParameters()
{
  ROS_INFO_STREAM("Getting kinematic parameters from parameter server.");

  // Using the full parameter name at the moment because the leading slash
  // in front of robot_description_kinematics is missing
  // in lookupParam function, but I'm not sure if this is a bug or I do not
  // understand the interface.
  std::string prefix = "/robot_description_kinematics/" + group_name_ + "/";
  ros::NodeHandle nh;

  std::map<std::string, double> dh_parameters, dummy;
  if (!lookupParam(prefix + "kinematics_solver_dh_parameters", dh_parameters, dummy))
  {
    ROS_ERROR_STREAM("Failed to load dh parameters for ik solver.");
    return false;
  }

  bot_parameters_.a1 = dh_parameters["a1"];
  bot_parameters_.a2 = dh_parameters["a2"];
  bot_parameters_.a3 = dh_parameters["a3"];
  bot_parameters_.l1 = dh_parameters["l1"];
  bot_parameters_.l2 = dh_parameters["l2"];
  bot_parameters_.l3 = dh_parameters["l3"];
  bot_parameters_.t1 = dh_parameters["t1"];
  bot_parameters_.t3 = dh_parameters["t3"];

  ROS_INFO_STREAM("Loaded parameters for ik solver:\n" << bot_parameters_);

  return true;
}

double MoveItBotKinematicsPlugin::distance(const std::vector<double>& a, const std::vector<double>& b) const
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += std::abs(b[i] - a[i]);
  return cost;
}

// Compute the index of the closest joint pose in 'candidates' from 'target'
std::size_t MoveItBotKinematicsPlugin::closestJointPose(const std::vector<double>& target,
                                                        const std::vector<std::vector<double>>& candidates) const
{
  size_t closest = 0;  // index into candidates
  double lowest_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    assert(target.size() == candidates[i].size());
    double c = distance(target, candidates[i]);
    if (c < lowest_cost)
    {
      closest = i;
      lowest_cost = c;
    }
  }
  return closest;
}

bool MoveItBotKinematicsPlugin::getAllIK(const Eigen::Affine3d& pose,
                                         std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();

  // Transform input pose
  // needed if we introduce a tip frame different from tool0
  // or a different base frame
  // Eigen::Affine3d tool_pose = diff_base.inverse() * pose *
  // tip_frame.inverse();

  // convert Eigen::Affine3d to Eigen::Isometry3d for bot_kinematics
  Eigen::Isometry3d pose_isometry;
  pose_isometry = pose.matrix();

  std::array<double, 3> sols;
  bot_kinematics::inverse(bot_parameters_, pose_isometry, sols.data());

  // Check the output
  std::vector<double> tmp(3);  // temporary storage for API reasons
    double* sol = sols.data();
    if (bot_kinematics::isValid(sol))
    {
      bot_kinematics::harmonizeTowardZero(sol);

      // TODO: make this better...
      std::copy(sol, sol + 3, tmp.data());
      // if (isValid(tmp))
      // {
      joint_poses.push_back(tmp);
      // }
    }

  return joint_poses.size() > 0;
}

bool MoveItBotKinematicsPlugin::getIK(const Eigen::Affine3d& pose, const std::vector<double>& seed_state,
                                      std::vector<double>& joint_pose) const
{
  // Descartes Robot Model interface calls for 'closest' point to seed position
  std::vector<std::vector<double>> joint_poses;
  if (!getAllIK(pose, joint_poses))
    return false;
  // Find closest joint pose; getAllIK() does isValid checks already
  joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
  return true;
}

}  // namespace moveit_bot_kinematics_plugin
