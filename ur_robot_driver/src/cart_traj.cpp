#include <ros/ros.h>
#include <cartesian_control_msgs/FollowCartesianTrajectoryAction.h>
#include <cartesian_control_msgs/CartesianTrajectory.h>
#include <actionlib/client/simple_action_client.h>

#include <Eigen/Dense>

void fill (cartesian_control_msgs::CartesianTrajectoryPoint& cartesian_state, double time)
{
  double omega = 0.4;
  double angle = sin(omega * time);

  // Compute orientation around z
  Eigen::Matrix3d rot_z = Eigen::Matrix3d::Zero();
  rot_z(0,0) = cos(angle);
  rot_z(0,1) = -sin(angle);
  rot_z(0,1) = 0;
  rot_z(1,0) = sin(angle);
  rot_z(1,1) = cos(angle); 
  rot_z(1,2) = 0;
  rot_z(2,0) = 0;
  rot_z(2,1) = 0;
  rot_z(2,2) = 1;
  Eigen::Matrix3d rot; 
  rot(0,0) = 0.0342088;
  rot(0,1) =  0.9994147;
  rot(0,2) = 0.0001144;
  rot(1,0) = 0.9994147;
  rot(1,1) = -0.0342088;
  rot(1,2) = -0.0001184;
  rot(2,0) = -0.0001144;
  rot(2,1) =  0.0001184;
  rot(2,2) =-1.0000000;
  Eigen::Matrix3d target_ori = rot * rot_z;
  Eigen::Quaterniond q(target_ori);

  cartesian_state.pose.position.x = -0.14397;
  cartesian_state.pose.position.y = -0.43562;
  cartesian_state.pose.position.z = 0.20203;

  cartesian_state.pose.orientation.w = q.w();
  cartesian_state.pose.orientation.x = q.x();
  cartesian_state.pose.orientation.y = q.y();
  cartesian_state.pose.orientation.z = q.z();

  cartesian_state.twist.linear.x = 0.0;
  cartesian_state.twist.linear.y = 0.0;
  cartesian_state.twist.linear.z = 0.0;

  cartesian_state.twist.angular.x = 0.0;
  cartesian_state.twist.angular.y = 0.0;
  cartesian_state.twist.angular.z  = -1 * omega * cos(omega * time);

  cartesian_state.acceleration.linear.x = 0.0;
  cartesian_state.acceleration.linear.y = 0.0;
  cartesian_state.acceleration.linear.z = 0.0;

  cartesian_state.acceleration.angular.x = 0.0;
  cartesian_state.acceleration.angular.y = 0.0;
  cartesian_state.acceleration.angular.z = -1 * (-omega * omega) * sin(omega * time);
  cartesian_state.time_from_start.fromSec(time);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "execute_traj");
  ros::NodeHandle nh;

  cartesian_control_msgs::FollowCartesianTrajectoryGoal goal;
  cartesian_control_msgs::CartesianTrajectoryPoint point;

  point.pose.position.x = -0.14397;
  point.pose.position.y = -0.43562;
  point.pose.position.z = 0.20203;
  point.pose.orientation.x = 0.7190997;
  point.pose.orientation.y = 0.6949069;
  point.pose.orientation.z = 0.0;
  point.pose.orientation.w = 0.0000823;

  point.twist.linear.x = 0.0;
  point.twist.linear.y = 0.0;
  point.twist.linear.z = 0.0;
  point.twist.angular.x = 0.0;
  point.twist.angular.y = 0.0;
  point.twist.angular.z = 0.0;

  point.acceleration.linear.x = 0.0;
  point.acceleration.linear.y = 0.0;
  point.acceleration.linear.z = 0.0;
  point.acceleration.angular.x = 0.0;
  point.acceleration.angular.y = 0.0;
  point.acceleration.angular.z = 0.0;
  point.time_from_start.fromSec(0.5);
  goal.trajectory.points.push_back(point);

  fill(point, 2.0);
  goal.trajectory.points.push_back(point);

  fill(point, 3.5);
  goal.trajectory.points.push_back(point);

  fill(point, 4.0);
  goal.trajectory.points.push_back(point);

  fill(point, 4.5);
  goal.trajectory.points.push_back(point);

  fill(point, 5.0);
  goal.trajectory.points.push_back(point);

  fill(point, 6.5);
  goal.trajectory.points.push_back(point);


  actionlib::SimpleActionClient<cartesian_control_msgs::FollowCartesianTrajectoryAction> ac("/pose_based_cartesian_traj_controller/follow_cartesian_trajectory", true);
  ac.waitForServer();
  std::cout << "Done waiting for server" << std::endl;
  ac.sendGoal(goal);
  bool res = ac.waitForResult();
  std::cout << res << std::endl;
}
