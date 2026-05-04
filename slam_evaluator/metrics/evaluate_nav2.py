#!/usr/bin/env python3
import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node


def create_pose(navigator, x, y, theta):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    # Simple Euler to Quaternion for Z rotation
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)
    return pose


class Nav2Evaluator(Node):
    def __init__(self, start_poses, goal_poses, output_csv):
        super().__init__("nav2_evaluator")
        self.navigator = BasicNavigator()
        self.start_poses = start_poses
        self.goal_poses = goal_poses
        self.output_csv = output_csv

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

    def set_initial_pose(self, pose):
        init_pose = PoseWithCovarianceStamped()
        init_pose.header = pose.header
        init_pose.pose.pose = pose.pose
        # Set some covariance
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] = 0.068
        self.initial_pose_pub.publish(init_pose)
        self.navigator.setInitialPose(pose)

    def run_evaluation(self):
        # Set the very first initial pose so AMCL can initialize
        if self.start_poses:
            self.get_logger().info("Setting initial pose to bootstrap AMCL...")
            self.set_initial_pose(self.start_poses[0])

        # Wait for Nav2 to be fully active
        self.navigator.waitUntilNav2Active()

        results = []

        for i, (start, goal) in enumerate(zip(self.start_poses, self.goal_poses)):
            self.get_logger().info(f"Running test {i + 1}/{len(self.start_poses)}")

            # Set initial pose and wait for AMCL to converge (simulate by waiting)
            self.set_initial_pose(start)
            time.sleep(2.0)

            start_time = time.time()

            # Request navigation
            self.navigator.goToPose(goal)

            # Wait for completion
            initial_distance = None
            number_of_recoveries = 0

            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    if initial_distance is None:
                        initial_distance = feedback.distance_remaining
                    number_of_recoveries = feedback.number_of_recoveries
                time.sleep(0.5)
                # Here we could implement kidnapping for some tests

            result = self.navigator.getResult()
            end_time = time.time()
            duration = end_time - start_time

            success = result == TaskResult.SUCCEEDED

            results.append(
                {
                    "test_id": i,
                    "success": success,
                    "duration": duration,
                    "initial_distance": initial_distance if initial_distance else 0.0,
                    "recoveries": number_of_recoveries,
                    "result_code": result,
                }
            )

            self.get_logger().info(
                f"Test {i + 1} completed. Success: {success}, Time: {duration:.2f}s, Dist: {initial_distance}, Recov: {number_of_recoveries}"
            )

        # Write to CSV
        with open(self.output_csv, mode="w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "test_id",
                    "success",
                    "duration",
                    "initial_distance",
                    "recoveries",
                    "result_code",
                ],
            )
            writer.writeheader()
            for r in results:
                writer.writerow(r)

        self.get_logger().info(
            f"Evaluation finished. Results saved to {self.output_csv}"
        )

    def run_kidnapping_evaluation(self, start, goal):
        self.get_logger().info("Running kidnapping evaluation...")
        self.set_initial_pose(start)
        time.sleep(2.0)

        self.navigator.goToPose(goal)

        kidnapped = False
        while not self.navigator.isTaskComplete():
            time.sleep(0.5)
            # Kidnap the robot halfway through
            if (
                not kidnapped
                and self.navigator.getFeedback()
                and self.navigator.getFeedback().distance_remaining < 5.0
            ):
                self.get_logger().info("KIDNAPPING ROBOT!")
                wrong_pose = create_pose(
                    self.navigator,
                    start.pose.position.x + 2.0,
                    start.pose.position.y - 2.0,
                    0,
                )
                self.set_initial_pose(wrong_pose)
                kidnapped = True

        result = self.navigator.getResult()
        success = result == TaskResult.SUCCEEDED
        self.get_logger().info(f"Kidnapping test complete. Success: {success}")


def main():
    parser = argparse.ArgumentParser(description="Nav2 Evaluator")
    parser.add_argument(
        "--output_csv", default="nav2_metrics.csv", help="Output CSV file"
    )
    parser.add_argument(
        "--poses_json", help="Path to JSON file with start and goal poses"
    )
    args, unknown = parser.parse_known_args()

    rclpy.init()

    nav = BasicNavigator()

    start_poses = []
    goal_poses = []

    if args.poses_json and Path(args.poses_json).exists():
        with open(args.poses_json, "r") as f:
            data = json.load(f)
            for item in data:
                s = item["start"]
                g = item["goal"]
                start_poses.append(create_pose(nav, s[0], s[1], s[2]))
                goal_poses.append(create_pose(nav, g[0], g[1], g[2]))
    else:
        # Default fallback poses
        start_poses = [create_pose(nav, 0.0, 0.0, 0.0), create_pose(nav, 1.0, 0.0, 0.0)]
        goal_poses = [create_pose(nav, 5.0, 0.0, 0.0), create_pose(nav, 0.0, 0.0, 3.14)]

    evaluator = Nav2Evaluator(start_poses, goal_poses, args.output_csv)

    try:
        evaluator.run_evaluation()
    except KeyboardInterrupt:
        pass
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
