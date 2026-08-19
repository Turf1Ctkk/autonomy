#!/usr/bin/env python3

import importlib.util
import math
import os
import sys
import types
import unittest

import numpy as np
import rospy
from nav_msgs.msg import Odometry


# The math test is also runnable in a px4ctrl-only catkin whitelist build,
# where traj_utils' generated Python module may not be present even though its
# C++ artifacts come from an underlay. Use the real message whenever available.
try:
    from traj_utils.msg import PolyTraj  # noqa: F401
except ImportError:
    class PolyTraj:
        def __init__(self):
            self.drone_id = 0
            self.traj_id = 0
            self.start_time = rospy.Time()
            self.order = 0
            self.coef_x = []
            self.coef_y = []
            self.coef_z = []
            self.duration = []

    traj_utils_module = types.ModuleType("traj_utils")
    traj_utils_msg_module = types.ModuleType("traj_utils.msg")
    traj_utils_msg_module.PolyTraj = PolyTraj
    traj_utils_module.msg = traj_utils_msg_module
    sys.modules["traj_utils"] = traj_utils_module
    sys.modules["traj_utils.msg"] = traj_utils_msg_module


SCRIPT_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "publish_polytraj.py",
)
SPEC = importlib.util.spec_from_file_location("publish_polytraj", SCRIPT_PATH)
PUBLISHER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PUBLISHER)


def evaluate_piece(coefficients, time, derivative):
    polynomial = np.asarray(coefficients, dtype=float)
    for _ in range(derivative):
        polynomial = np.polyder(polynomial)
    return float(np.polyval(polynomial, time))


class PolyTrajPublisherTest(unittest.TestCase):
    def make_args(self, shape):
        return types.SimpleNamespace(
            shape=shape,
            radius=1.2,
            eight_width=0.8,
            speed=0.6,
            laps=1,
            pieces_per_lap=32,
            turn="left",
            z=None,
            drone_id=0,
            traj_id=7,
        )

    def make_odometry(self):
        odometry = Odometry()
        odometry.pose.pose.position.x = 2.0
        odometry.pose.pose.position.y = -1.0
        odometry.pose.pose.position.z = 1.5
        yaw = 0.7
        odometry.pose.pose.orientation.w = math.cos(0.5 * yaw)
        odometry.pose.pose.orientation.z = math.sin(0.5 * yaw)
        return odometry, yaw

    def axis_piece(self, coefficients, piece):
        begin = piece * 6
        return coefficients[begin:begin + 6]

    def sample_local_positions(self, message, yaw):
        axes = (message.coef_x, message.coef_y, message.coef_z)
        start = np.array([
            evaluate_piece(self.axis_piece(axis, 0), 0.0, 0) for axis in axes
        ])
        forward = np.array([math.cos(yaw), math.sin(yaw)])
        left = np.array([-math.sin(yaw), math.cos(yaw)])
        samples = []
        for piece, duration in enumerate(message.duration):
            for time in np.linspace(0.0, duration, 21):
                position = np.array([
                    evaluate_piece(self.axis_piece(axis, piece), time, 0)
                    for axis in axes
                ])
                displacement = position[:2] - start[:2]
                samples.append([np.dot(displacement, forward),
                                np.dot(displacement, left)])
        return np.asarray(samples)

    def test_circle_starts_at_odometry_and_is_c2_closed(self):
        args = self.make_args("circle")
        odometry, yaw = self.make_odometry()
        message, _ = PUBLISHER.build_polytraj(args, odometry, rospy.Time(10.0))

        self.assertEqual(message.order, 5)
        self.assertEqual(len(message.duration), args.pieces_per_lap)
        self.assertAlmostEqual(sum(message.duration),
                               2.0 * math.pi * args.radius / args.speed, places=5)

        axes = (message.coef_x, message.coef_y, message.coef_z)
        expected_start = (2.0, -1.0, 1.5)
        for coefficients, expected in zip(axes, expected_start):
            self.assertAlmostEqual(evaluate_piece(self.axis_piece(coefficients, 0),
                                                  0.0, 0), expected, places=5)

        initial_velocity = np.array([
            evaluate_piece(self.axis_piece(axis, 0), 0.0, 1) for axis in axes
        ])
        expected_velocity = args.speed * np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        np.testing.assert_allclose(initial_velocity, expected_velocity, atol=1.0e-5)

        local_positions = self.sample_local_positions(message, yaw)
        self.assertGreaterEqual(np.min(local_positions[:, 0]), -1.0e-4)
        self.assertAlmostEqual(np.max(local_positions[:, 0]),
                               2.0 * args.radius, places=3)
        self.assertAlmostEqual(np.min(local_positions[:, 1]), -args.radius, places=3)
        self.assertAlmostEqual(np.max(local_positions[:, 1]), args.radius, places=3)

        for piece in range(len(message.duration) - 1):
            for axis in axes:
                left = self.axis_piece(axis, piece)
                right = self.axis_piece(axis, piece + 1)
                for derivative in range(3):
                    self.assertAlmostEqual(
                        evaluate_piece(left, message.duration[piece], derivative),
                        evaluate_piece(right, 0.0, derivative), places=3
                    )

        last_piece = len(message.duration) - 1
        endpoint = np.array([
            evaluate_piece(self.axis_piece(axis, last_piece),
                           message.duration[last_piece], 0) for axis in axes
        ])
        np.testing.assert_allclose(endpoint, expected_start, atol=1.0e-4)

    def test_figure_eight_stays_ahead_and_starts_at_requested_speed(self):
        args = self.make_args("eight")
        odometry, yaw = self.make_odometry()
        message, _ = PUBLISHER.build_polytraj(args, odometry, rospy.Time(10.0))
        axes = (message.coef_x, message.coef_y, message.coef_z)

        initial_velocity = np.array([
            evaluate_piece(self.axis_piece(axis, 0), 0.0, 1) for axis in axes
        ])
        expected_velocity = args.speed * np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        np.testing.assert_allclose(initial_velocity, expected_velocity, atol=1.0e-5)
        self.assertAlmostEqual(np.linalg.norm(initial_velocity), args.speed, places=5)

        local_positions = self.sample_local_positions(message, yaw)
        self.assertGreaterEqual(np.min(local_positions[:, 0]), -1.0e-4)
        self.assertAlmostEqual(np.max(local_positions[:, 0]),
                               2.0 * args.radius, places=3)
        self.assertAlmostEqual(np.min(local_positions[:, 1]),
                               -args.eight_width, places=3)
        self.assertAlmostEqual(np.max(local_positions[:, 1]),
                               args.eight_width, places=3)

        # With 32 pieces, theta=pi/2 and 3*pi/2 are exact nodes. Both are the
        # figure-eight crossing one radius ahead of the odometry start.
        for node in (args.pieces_per_lap // 4,
                     3 * args.pieces_per_lap // 4):
            crossing = np.array([
                evaluate_piece(self.axis_piece(axis, node), 0.0, 0)
                for axis in axes
            ])
            start = np.array([2.0, -1.0, 1.5])
            displacement = crossing[:2] - start[:2]
            forward = np.array([math.cos(yaw), math.sin(yaw)])
            left = np.array([-math.sin(yaw), math.cos(yaw)])
            self.assertAlmostEqual(np.dot(displacement, forward), args.radius, places=4)
            self.assertAlmostEqual(np.dot(displacement, left), 0.0, places=4)


if __name__ == "__main__":
    unittest.main()
