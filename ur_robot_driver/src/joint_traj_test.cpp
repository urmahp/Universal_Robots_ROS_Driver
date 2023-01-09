
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <Eigen/Dense>

void fill(trajectory_msgs::JointTrajectoryPoint& point, double time)
{
  double omega = 0.4;
  double angle = sin(omega * time);
  std::cout << angle << std::endl;
  double vel = omega * cos(omega * time);
  double acc = (-omega * omega) * sin(omega * time);
  point.positions.clear();
  point.positions.push_back(angle);
  point.positions.push_back(0.0);
  point.positions.push_back(0.0);
  point.positions.push_back(0.0);
  point.positions.push_back(0.0);
  point.positions.push_back(0.0);

  point.velocities.clear();
  point.velocities.push_back(vel);
  point.velocities.push_back(0.0);
  point.velocities.push_back(0.0);
  point.velocities.push_back(0.0);
  point.velocities.push_back(0.0);
  point.velocities.push_back(0.0);

  point.accelerations.clear();
  point.accelerations.push_back(acc);
  point.accelerations.push_back(0.0);
  point.accelerations.push_back(0.0);
  point.accelerations.push_back(0.0);
  point.accelerations.push_back(0.0);
  point.accelerations.push_back(0.0);

  point.time_from_start.fromSec(time);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "execute_traj");
  ros::NodeHandle nh;

  trajectory_msgs::JointTrajectoryPoint point;
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                  "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint" };

  fill(point, 3.0);
  goal.trajectory.points.push_back(point);

  fill(point, 5.0);
  goal.trajectory.points.push_back(point);

  fill(point, 6.0);
  goal.trajectory.points.push_back(point);

  fill(point, 7.0);
  goal.trajectory.points.push_back(point);

  fill(point, 8.0);
  goal.trajectory.points.push_back(point);

  fill(point, 9.5);
  goal.trajectory.points.push_back(point);

  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/forward_joint_traj_controller/"
                                                                              "follow_joint_trajectory",
                                                                              true);
  ac.waitForServer();
  ac.sendGoal(goal);
  ac.waitForResult();
}