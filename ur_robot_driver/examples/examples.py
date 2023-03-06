#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    FollowCartesianTrajectoryResult,
    CartesianTrajectoryPoint)
import geometry_msgs.msg

from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllersRequest, ListControllers


class Robot:
    def __init__(self):
        self.timeout = rospy.Duration(10)
        self.all_controllers = ["scaled_pos_joint_traj_controller",
                                "pos_joint_traj_controller",
                                "scaled_vel_joint_traj_controller",
                                "forward_joint_traj_controller",
                                "forward_cartesian_traj_controller",
                                "vel_joint_traj_controller",
                                "joint_group_vel_controller",
                                "twist_controller",
                                "pose_based_cartesian_traj_controller",
                                "joint_based_cartesian_traj_controller",
                                ]
        self.init_robot()

    def init_robot(self):
        self.switch_controllers_client = rospy.ServiceProxy('/controller_manager/switch_controller',
                                                            SwitchController)
        try:
            self.switch_controllers_client.wait_for_service(self.timeout)
        except rospy.exceptions.ROSException as err:
            raise Exception(
                "Could not reach controller switch service. Make sure that the driver is actually running.")

        self.list_controllers_client = rospy.ServiceProxy('/controller_manager/list_controllers',
                                                          ListControllers)
        try:
            self.list_controllers_client.wait_for_service(self.timeout)
        except rospy.exceptions.ROSException as err:
            raise Exception(
                "Could not reach controller switch service. Make sure that the driver is actually running.")

        self.load_controller_client = rospy.ServiceProxy('/controller_manager/load_controller',
                                                         LoadController)
        try:
            self.load_controller_client.wait_for_service(self.timeout)
        except rospy.exceptions.ROSException as err:
            raise Exception(
                "Could not reach controller switch service. Make sure that the driver is actually running.")

    def send_joint_trajectory(self, waypts, time_vec):
        """Execute joint trajectory on the robot, using scaled position trajectory controller."""
        if len(waypts) != len(time_vec):
            raise Exception(
                "waypoints vector and time vec should be same length")

        self.switch_on_controller("scaled_pos_joint_traj_controller")

        joint_trajectory_client = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        if not joint_trajectory_client.wait_for_server(self.timeout):
            raise Exception(
                "Could not reach controller action. Make sure that the driver is actually running.")

        # Create joint trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        for i in range(len(waypts)):
            point = JointTrajectoryPoint()
            point.positions = waypts[i]
            point.time_from_start = rospy.Duration(time_vec[i])
            goal.trajectory.points.append(point)

        # Publish trajectory and wait for result
        joint_trajectory_client.send_goal(goal)
        joint_trajectory_client.wait_for_result()

        return joint_trajectory_client.get_result().error_code == FollowJointTrajectoryResult.SUCCESSFUL

    def send_joint_vel_trajectory(self, waypts, time_vec):
        """Execute joint trajectory on the robot, using scaled velocity trajectory controller."""
        if len(waypts) != len(time_vec):
            raise Exception(
                "waypoints vector and time vec should be same length")

        self.switch_on_controller("scaled_vel_joint_traj_controller")

        joint_trajectory_client = actionlib.SimpleActionClient(
            '/scaled_vel_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        if not joint_trajectory_client.wait_for_server(self.timeout):
            raise Exception(
                "Could not reach controller action. Make sure that the driver is actually running.")

        # Create joint trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        for i in range(len(waypts)):
            point = JointTrajectoryPoint()
            point.positions = waypts[i]
            point.time_from_start = rospy.Duration(time_vec[i])
            goal.trajectory.points.append(point)

        # Publish trajectory and wait for result
        joint_trajectory_client.send_goal(goal)
        joint_trajectory_client.wait_for_result()

        return joint_trajectory_client.get_result().error_code == FollowJointTrajectoryResult.SUCCESSFUL

    def send_joint_passthrough_trajectory(self, waypts, time_vec):
        """Execute joint trajectory on the robot, using joint passthrough controller."""
        if len(waypts) != len(time_vec):
            raise Exception(
                "waypoints vector and time vec should be same length")

        self.switch_on_controller("forward_joint_traj_controller")

        joint_passthrough_trajectory_client = actionlib.SimpleActionClient(
            '/forward_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        if not joint_passthrough_trajectory_client.wait_for_server(self.timeout):
            raise Exception(
                "Could not reach joint passthrough controller action. Make sure that the driver is actually running.")

        # Create joint trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        for i in range(len(waypts)):
            point = JointTrajectoryPoint()
            point.positions = waypts[i]
            point.time_from_start = rospy.Duration(time_vec[i])
            goal.trajectory.points.append(point)

        # Publish trajectory and wait for result
        joint_passthrough_trajectory_client.send_goal(goal)
        joint_passthrough_trajectory_client.wait_for_result()

        return joint_passthrough_trajectory_client.get_result().error_code == FollowJointTrajectoryResult.SUCCESSFUL

    def send_cartesian_pose_trajectory(self, waypts, time_vec):
        """Execute cartesian trajectory on the robot, using pose based cartesian controller."""
        if len(waypts) != len(time_vec):
            raise Exception(
                "waypoints vector and time vec should be same length")

        self.switch_on_controller("pose_based_cartesian_traj_controller")

        cartesian_trajectory_client = actionlib.SimpleActionClient(
            '/pose_based_cartesian_traj_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        if not cartesian_trajectory_client.wait_for_server(self.timeout):
            raise Exception(
                "Could not reach cartesian controller action. Make sure that the driver is actually running.")

        # Create cartesian trajectory
        goal = FollowCartesianTrajectoryGoal()
        for i in range(len(waypts)):
            point = CartesianTrajectoryPoint()
            point.pose = geometry_msgs.msg.Pose(
                geometry_msgs.msg.Vector3(
                    waypts[i][0], waypts[i][1], waypts[i][2]),
                geometry_msgs.msg.Quaternion(
                    waypts[i][3], waypts[i][4], waypts[i][5], waypts[i][6]))
            point.time_from_start = rospy.Duration(time_vec[i])
            goal.trajectory.points.append(point)

        # Publish trajectory and wait for result
        cartesian_trajectory_client.send_goal(goal)
        cartesian_trajectory_client.wait_for_result()

        return cartesian_trajectory_client.get_result().error_code == FollowCartesianTrajectoryResult.SUCCESSFUL

    def send_cartesian_joint_trajectory(self, waypts, time_vec):
        """Execute cartesian trajectory on the robot, using joint based cartesian controller."""
        if len(waypts) != len(time_vec):
            raise Exception(
                "waypoints vector and time vec should be same length")

        self.switch_on_controller("joint_based_cartesian_traj_controller")

        cartesian_trajectory_client = actionlib.SimpleActionClient(
            '/joint_based_cartesian_traj_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        if not cartesian_trajectory_client.wait_for_server(self.timeout):
            raise Exception(
                "Could not reach cartesian controller action. Make sure that the driver is actually running.")

        # Create cartesian trajectory
        goal = FollowCartesianTrajectoryGoal()
        for i in range(len(waypts)):
            point = CartesianTrajectoryPoint()
            point.pose = geometry_msgs.msg.Pose(
                geometry_msgs.msg.Vector3(
                    waypts[i][0], waypts[i][1], waypts[i][2]),
                geometry_msgs.msg.Quaternion(
                    waypts[i][3], waypts[i][4], waypts[i][5], waypts[i][6]))
            point.time_from_start = rospy.Duration(time_vec[i])
            goal.trajectory.points.append(point)

        # Publish trajectory and wait for result
        cartesian_trajectory_client.send_goal(goal)
        cartesian_trajectory_client.wait_for_result()

        return cartesian_trajectory_client.get_result().error_code == FollowJointTrajectoryResult.SUCCESSFUL

    def send_cartesian_passthrough_trajectory(self, waypts, time_vec):
        """Execute cartesian trajectory on the robot, using cartesian passthrough controller."""
        if len(waypts) != len(time_vec):
            raise Exception(
                "waypoints vector and time vec should be same length")

        self.switch_on_controller("forward_cartesian_traj_controller")

        cartesian_trajectory_client = actionlib.SimpleActionClient(
            '/forward_cartesian_traj_controller/follow_cartesian_trajectory', FollowCartesianTrajectoryAction)
        if not cartesian_trajectory_client.wait_for_server(self.timeout):
            raise Exception(
                "Could not reach cartesian controller action. Make sure that the driver is actually running.")

        # Create cartesian trajectory
        goal = FollowCartesianTrajectoryGoal()
        for i in range(len(waypts)):
            point = CartesianTrajectoryPoint()
            point.pose = geometry_msgs.msg.Pose(
                geometry_msgs.msg.Vector3(
                    waypts[i][0], waypts[i][1], waypts[i][2]),
                geometry_msgs.msg.Quaternion(
                    waypts[i][3], waypts[i][4], waypts[i][5], waypts[i][6]))
            point.time_from_start = rospy.Duration(time_vec[i])
            goal.trajectory.points.append(point)

        # Publish trajectory and wait for result
        cartesian_trajectory_client.send_goal(goal)
        cartesian_trajectory_client.wait_for_result()

        return cartesian_trajectory_client.get_result().error_code == FollowJointTrajectoryResult.SUCCESSFUL

    def send_twist_command(self, twist, duration):
        """send twist command to the robot, using twist_controller controller."""
        if len(twist) != 6:
            raise Exception("Twist should have length 6")
        self.switch_on_controller("twist_controller")

        twist_msg = geometry_msgs.msg.Twist()
        twist_msg.linear.x = twist[0]
        twist_msg.linear.y = twist[1]
        twist_msg.linear.z = twist[2]
        twist_msg.angular.x = twist[3]
        twist_msg.angular.y = twist[4]
        twist_msg.angular.z = twist[5]

        twist_pub = rospy.Publisher(
            "/twist_controller/command", geometry_msgs.msg.Twist, queue_size=1)

        # publish twist
        rate = rospy.Rate(10)
        step_time = 0.1
        cur_time = 0
        while not rospy.is_shutdown():
            if cur_time > duration:
                # Stop the robot
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.0
                twist_pub.publish(twist_msg)
                break
            twist_pub.publish(twist_msg)
            cur_time += step_time
            rate.sleep()

    def switch_on_controller(self, controller_name):
        """Switches on the given controller stopping all other known controllers with best_effort
        strategy."""

        # If the controller is not part of the listed controllers we need to load it first
        srv_list_controllers = ListControllersRequest()
        result = self.list_controllers_client(srv_list_controllers)
        controller_found = False
        for i in range(len(result.controller)):
            if result.controller[i].name == controller_name:
                controller_found = True
                break

        if not controller_found:
            srv_load_controller = LoadControllerRequest()
            srv_load_controller.name = controller_name
            result = self.load_controller_client(srv_load_controller)
            if result.ok == False:
                raise Exception(
                    "Failed to load controller {}".format(controller_name))

        # Switch controller
        srv = SwitchControllerRequest()
        srv.stop_controllers = self.all_controllers
        srv.start_controllers = [controller_name]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        result = self.switch_controllers_client(srv)
        if result.ok == False:
            raise Exception(
                "Failed to switch on controller {}".format(controller_name))


if __name__ == '__main__':

    rospy.init_node('robot_control')
    robot = Robot()

    # Available controllers:
    #   scaled_pos_joint_traj_controller
    #   scaled_vel_joint_traj_controller
    #   forward_joint_traj_controller
    #   forward_cartesian_traj_controller
    #   pose_based_cartesian_traj_controller
    #   joint_based_cartesian_traj_controller
    #   twist_controller

    # Change according to the controller you want to use
    controller_type = "scaled_pos_joint_traj_controller"

    if controller_type == "scaled_pos_joint_traj_controller":
        # The following list are arbitrary joint positions, change according to your own needs
        waypts = [[-1.6006, -1.7272, -2.2030, -0.8079, 1.5951, -0.0311],
                  [-1.2, -1.4, -1.9, -1.2, 1.5951, -0.0311],
                  [-1.6006, -1.7272, -2.2030, -0.8079, 1.5951, -0.0311]]
        time_vec = [4.0, 8.0, 12]

        # Send trajectory to the robot
        robot.send_joint_trajectory(waypts, time_vec)

    elif controller_type == "scaled_vel_joint_traj_controller":
        # The following list are arbitrary joint positions, change according to your own needs
        waypts = [[-1.6006, -1.7272, -2.2030, -0.8079, 1.5951, -0.0311],
                  [-1.2, -1.4, -1.9, -1.2, 1.5951, -0.0311],
                  [-1.6006, -1.7272, -2.2030, -0.8079, 1.5951, -0.0311]]
        time_vec = [4.0, 8.0, 12]

        # Send trajectory to the robot
        robot.send_joint_vel_trajectory(waypts, time_vec)

    elif controller_type == "forward_joint_traj_controller":
        # The following list are arbitrary joint positions, change according to your own needs
        waypts = [[-1.6006, -1.7272, -2.2030, -0.8079, 1.5951, -0.0311],
                  [-1.2, -1.4, -1.9, -1.2, 1.5951, -0.0311],
                  [-1.6006, -1.7272, -2.2030, -0.8079, 1.5951, -0.0311]]
        time_vec = [4.0, 8.0, 12]

        # Send trajectory to the robot
        robot.send_joint_passthrough_trajectory(waypts, time_vec)

    elif controller_type == "forward_cartesian_traj_controller":
        # The following list are arbitrary TCP positions, change according to your own needs
        # First three are position x, y, z and last 4 are orientation as quaternion x, y, z, w
        waypts = [[0.143, -0.435, 0.202, 0, -0.9998442, -0.0126323, -0.0123297],
                  [0.183, -0.415, 0.402, 0, -0.9998442, -0.0126323, -0.0123297],
                  [0.143, -0.435, 0.202, 0, -0.9998442, -0.0126323, -0.0123297]]
        time_vec = [4.0, 8.0, 12]

        # Send trajectory to the robot
        robot.send_cartesian_passthrough_trajectory(waypts, time_vec)

    elif controller_type == "pose_based_cartesian_traj_controller":
        # The following list are arbitrary TCP positions, change according to your own needs
        # First three are position x, y, z and last 4 are orientation as quaternion x, y, z, w
        waypts = [[0.143, -0.435, 0.202, 0, -0.9998442, -0.0126323, -0.0123297],
                  [0.183, -0.415, 0.402, 0, -0.9998442, -0.0126323, -0.0123297],
                  [0.143, -0.435, 0.202, 0, -0.9998442, -0.0126323, -0.0123297]]
        time_vec = [4.0, 8.0, 12]

        # Send trajectory to the robot
        robot.send_cartesian_pose_trajectory(waypts, time_vec)

    elif controller_type == "joint_based_cartesian_traj_controller":
        # The following list are arbitrary TCP positions, change according to your own needs
        # First three are position x, y, z and last 4 are orientation as quaternion x, y, z, w
        waypts = [[0.143, -0.435, 0.202, 0, -0.9998442, -0.0126323, -0.0123297],
                  [0.183, -0.415, 0.402, 0, -0.9998442, -0.0126323, -0.0123297],
                  [0.143, -0.435, 0.202, 0, -0.9998442, -0.0126323, -0.0123297]]
        time_vec = [4.0, 8.0, 12]

        # Send trajectory to the robot
        robot.send_cartesian_joint_trajectory(waypts, time_vec)

    elif controller_type == "twist_controller":
        # The following list is an arbitrary TCP velocity, change according to your own needs
        # Twist is [x, y, z, rx, ry, rz]
        twist = [0, 0, 0.1, 0, 0, 0]
        duration = 2
        robot.send_twist_command(twist, duration)
