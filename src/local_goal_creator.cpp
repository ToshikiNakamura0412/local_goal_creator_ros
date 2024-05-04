/**
 * @file local_goal_creator.cpp
 * @author Toshiki Nakamura
 * @brief C++ implementation of local goal creator
 * @date 2024-05-04
 * @copyright Copyright (c) 2024
 */

#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator() : private_nh_("~")
{
  private_nh_.param<int>("hz", hz_, 10);
  private_nh_.param<float>("target_dist_to_goal", target_dist_to_goal_, 0.5);

  goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("local_goal", 1);
  path_sub_ = nh_.subscribe("/path", 1, &LocalGoalCreator::path_callback, this);
  pose_sub_ = nh_.subscribe("/robot_pose", 1, &LocalGoalCreator::pose_callback, this);

  ROS_INFO_STREAM(ros::this_node::getName() << "node has started..");
  ROS_INFO_STREAM("hz: " << hz_);
  ROS_INFO_STREAM("target_dist_to_goal: " << target_dist_to_goal_);
}

void LocalGoalCreator::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){ robot_pose_ = *msg; }

void LocalGoalCreator::path_callback(const nav_msgs::Path::ConstPtr &msg) { path_ = *msg; }

void LocalGoalCreator::process()
{
  ros::Rate loop_rate(hz_);
  while (ros::ok())
  {
    ros::spinOnce();

    if (robot_pose_.has_value() && path_.has_value())
    {
      geometry_msgs::PoseStamped goal_pose;
      goal_pose = create_goal(robot_pose_.value(), path_.value());
      goal_pose.header.stamp = ros::Time::now();
      goal_pose.header.frame_id = robot_pose_.value().header.frame_id;
      goal_pub_.publish(goal_pose);
    }

    loop_rate.sleep();
  }
}

geometry_msgs::PoseStamped LocalGoalCreator::create_goal(const geometry_msgs::PoseWithCovarianceStamped &robot_pose, const nav_msgs::Path &path)
{
  geometry_msgs::PoseStamped goal_pose;
  for (int i = 1; 0 < i && i < path.poses.size(); i++)
  {
    if (calc_dist_between_points(robot_pose.pose.pose.position, path.poses[i].pose.position) >= target_dist_to_goal_)
    {
      goal_pose.pose.position = path.poses[i].pose.position;
      goal_pose.pose.orientation = calc_direction(path.poses[i-1].pose.position, path.poses[i].pose.position);
      return goal_pose;
    }
  }
  return path.poses.back();
}

float LocalGoalCreator::calc_dist_between_points(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
  const float dx = point1.x - point2.x;
  const float dy = point1.y - point2.y;
  return hypot(dx, dy);
}

geometry_msgs::Quaternion LocalGoalCreator::calc_direction(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2)
{
  const float yaw = atan2(point2.y - point1.y, point2.x - point1.x);
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);

  geometry_msgs::Quaternion q_msg;
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  return q_msg;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "obstacle_inflater");
  LocalGoalCreator local_goal_creator;
  local_goal_creator.process();

  return 0;
}
