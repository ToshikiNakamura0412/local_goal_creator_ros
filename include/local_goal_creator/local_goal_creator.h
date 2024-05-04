/**
 * @file local_goal_creator.h
 * @author Toshiki Nakamura
 * @brief C++ implementation of local goal creator
 * @date 2024-05-04
 * @copyright Copyright (c) 2024
 */

#ifndef LOCAL_GOAL_CREATOR_LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_LOCAL_GOAL_CREATOR_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <optional>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

class LocalGoalCreator
{
public:
  LocalGoalCreator();
  void process();

private:
  void path_callback(const nav_msgs::Path::ConstPtr &msg);
  void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  geometry_msgs::PoseStamped create_goal(const geometry_msgs::PoseWithCovarianceStamped &robot_pose, const nav_msgs::Path &path);
  float calc_dist_between_points(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2);
  geometry_msgs::Quaternion calc_direction(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2);

  int hz_;
  float target_dist_to_goal_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher goal_pub_;
  ros::Subscriber path_sub_;
  ros::Subscriber pose_sub_;

  std::optional<geometry_msgs::PoseWithCovarianceStamped> robot_pose_;
  std::optional<nav_msgs::Path> path_;
};

#endif // LOCAL_GOAL_CREATOR_LOCAL_GOAL_CREATOR_H
