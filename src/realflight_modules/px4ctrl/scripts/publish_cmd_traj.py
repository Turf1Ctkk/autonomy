#!/usr/bin/env python3
"""
Publish a circle or figure-eight as quadrotor_msgs/PositionCommand for the
alg0 controller (pose_solver != 3).

Usage:
    rosrun px4ctrl publish_cmd_traj.py [--shape circle|eight] [--radius 1.5]
                                       [--speed 0.5] [--z 1.0] ...

The script waits for odometry, then continuously publishes PositionCommand
at a high rate so px4ctrl stays in CMD_CTRL mode.  The trajectory starts
from the vehicle's current position / yaw.

Suggested workflow:
  1. Launch px4ctrl with pose_solver=0 (alg0).
  2. Arm and trigger auto-takeoff (e.g. via /px4ctrl/takeoff_land).
  3. After the vehicle reaches 1 m and enters AUTO_HOVER, switch the RC
     command-mode switch to ON (or send the equivalent signal).
  4. Run this script.  px4ctrl transitions to CMD_CTRL and tracks the
     published trajectory.
  5. Ctrl+C to stop; the vehicle holds the last position and after 0.5 s
     without a command it falls back to AUTO_HOVER.
"""

import argparse
import math
import sys

import numpy as np
import rospy
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def positive_float(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite number > 0")
    return parsed


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be > 0")
    return parsed


def quaternion_yaw(quaternion):
    values = np.array(
        [quaternion.w, quaternion.x, quaternion.y, quaternion.z], dtype=float
    )
    norm = np.linalg.norm(values)
    if not np.isfinite(values).all() or norm < 1.0e-6:
        raise ValueError("odometry contains an invalid orientation quaternion")
    w, x, y, z = values / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ---------------------------------------------------------------------------
# curve geometry (same as publish_polytraj.py)
# ---------------------------------------------------------------------------

class PlanarCurve:
    """Curve position and its first two derivatives with respect to theta."""

    def __init__(self, shape, radius, eight_width, turn_sign):
        self.shape = shape
        self.radius = radius
        self.eight_width = eight_width
        self.turn_sign = turn_sign

    def evaluate(self, theta):
        if self.shape == "circle":
            sine = math.sin(theta)
            cosine = math.cos(theta)
            position = np.array(
                [self.radius * (1.0 - cosine),
                 self.turn_sign * self.radius * sine]
            )
            first = np.array(
                [self.radius * sine, self.turn_sign * self.radius * cosine]
            )
            second = np.array(
                [self.radius * cosine, -self.turn_sign * self.radius * sine]
            )
            return position, first, second

        sine = math.sin(theta)
        cosine = math.cos(theta)
        position = np.array(
            [self.radius * (1.0 - cosine),
             self.eight_width * math.sin(2.0 * theta)]
        )
        first = np.array(
            [self.radius * sine, 2.0 * self.eight_width * math.cos(2.0 * theta)]
        )
        second = np.array(
            [self.radius * cosine, -4.0 * self.eight_width * math.sin(2.0 * theta)]
        )
        return position, first, second


def constant_speed_state(curve, theta, speed):
    """Return (position, velocity, acceleration) at theta for a given speed."""
    position, first, second = curve.evaluate(theta)
    derivative_norm = np.linalg.norm(first)
    if derivative_norm < 1.0e-9:
        raise ValueError("curve parameterization has a zero tangent")

    theta_dot = speed / derivative_norm
    theta_ddot = -speed * speed * np.dot(first, second) / derivative_norm ** 4
    velocity = first * theta_dot
    acceleration = second * theta_dot * theta_dot + first * theta_ddot
    return position, velocity, acceleration


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def make_argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Publish a PositionCommand trajectory for the alg0 controller. "
            "Continuously sends circle or figure-eight waypoints at high rate."
        )
    )
    parser.add_argument("--shape", choices=("circle", "eight"), default="circle")
    parser.add_argument("--radius", type=positive_float, default=1.5,
                        help="circle radius or figure-eight forward half-span [m]")
    parser.add_argument("--eight-width", type=positive_float, default=1.0,
                        help="figure-eight lateral half-width [m] (default: radius)")
    parser.add_argument("--speed", type=positive_float, default=0.5,
                        help="constant tracking speed [m/s]")
    parser.add_argument("--z", type=float, default=1.2,
                        help="world-frame altitude [m] (default: 1.2)")
    parser.add_argument("--yaw", type=float, default=None,
                        help="fixed yaw angle [rad] (default: current odom yaw)")
    parser.add_argument("--laps", type=positive_int, default=10,
                        help="number of laps before looping back to the start")
    parser.add_argument("--turn", choices=("left", "right"), default="left",
                        help="circle initially turns left or right")
    parser.add_argument("--rate", type=positive_float, default=50.0,
                        help="PositionCommand publish rate [Hz]")
    parser.add_argument("--odom-topic", default="/lidar_slam/imu_propagate")
    parser.add_argument("--cmd-topic", default="/setpoints_cmd",
                        help="PositionCommand topic px4ctrl subscribes to")
    parser.add_argument("--odom-timeout", type=positive_float, default=5.0)
    parser.add_argument("--traj-id", type=positive_int, default=1)
    return parser


def main():
    args = make_argument_parser().parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("cmd_traj_publisher", anonymous=True)

    publisher = rospy.Publisher(args.cmd_topic, PositionCommand, queue_size=1)

    # --- wait for odometry ---
    rospy.loginfo("Waiting up to %.1f s for odometry on %s",
                  args.odom_timeout, args.odom_topic)
    try:
        odometry = rospy.wait_for_message(args.odom_topic, Odometry,
                                          timeout=args.odom_timeout)
    except rospy.ROSException as error:
        rospy.logfatal("No odometry received: %s", error)
        return 1

    current = odometry.pose.pose.position
    start_position = np.array(
        [current.x, current.y, current.z if args.z is None else args.z], dtype=float
    )
    if not np.isfinite(start_position).all():
        rospy.logfatal("Odometry contains a non-finite position")
        return 1

    yaw0 = args.yaw if args.yaw is not None else quaternion_yaw(odometry.pose.pose.orientation)
    world_rotation = np.array(
        [[math.cos(yaw0), -math.sin(yaw0)],
         [math.sin(yaw0), math.cos(yaw0)]],
        dtype=float,
    )
    turn_sign = 1.0 if args.turn == "left" else -1.0
    eight_width = args.eight_width if args.eight_width is not None else args.radius
    curve = PlanarCurve(args.shape, args.radius, eight_width, turn_sign)

    total_theta = 2.0 * math.pi * args.laps
    total_distance = 0.0
    # Pre-compute total arc length to know when the trajectory "ends"
    check_steps = 500
    for step in range(check_steps):
        t0 = total_theta * step / check_steps
        t1 = total_theta * (step + 1) / check_steps
        _, first, _ = curve.evaluate(0.5 * (t0 + t1))
        total_distance += np.linalg.norm(first) * (t1 - t0)
    total_duration = total_distance / args.speed

    rospy.loginfo(
        "Starting %s trajectory: radius=%.2f, speed=%.2f m/s, altitude=%.2f m, "
        "duration=%.1f s, yaw0=%.3f rad",
        args.shape, args.radius, args.speed, start_position[2],
        total_duration, yaw0,
    )

    # --- publish loop ---
    rate = rospy.Rate(args.rate)
    start_time = rospy.Time.now()
    period = 1.0 / args.rate

    # Keep a reference to the last published command so we can hold position
    # when the trajectory finishes (before the script exits).
    last_cmd = None

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()

        if elapsed < total_duration:
            theta = total_theta * elapsed / total_duration
            planar_pos, planar_vel, planar_acc = constant_speed_state(
                curve, theta, args.speed
            )
            position = start_position.copy()
            velocity = np.zeros(3)
            acceleration = np.zeros(3)
            position[:2] += world_rotation.dot(planar_pos)
            velocity[:2] = world_rotation.dot(planar_vel)
            acceleration[:2] = world_rotation.dot(planar_acc)
        else:
            # Hold the last position (hover at trajectory end point)
            if last_cmd is not None:
                publisher.publish(last_cmd)
                rate.sleep()
                continue
            # Fallback: compute the end point
            theta = total_theta
            planar_pos, planar_vel, planar_acc = constant_speed_state(
                curve, theta, args.speed
            )
            position = start_position.copy()
            velocity = np.zeros(3)
            acceleration = np.zeros(3)
            position[:2] += world_rotation.dot(planar_pos)

        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"
        cmd.trajectory_id = args.traj_id
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY

        cmd.position = Point(*position)
        cmd.velocity = Vector3(*velocity)
        cmd.acceleration = Vector3(*acceleration)
        cmd.jerk = Vector3(0.0, 0.0, 0.0)
        cmd.yaw = float(yaw0)
        cmd.yaw_dot = 0.0

        publisher.publish(cmd)
        last_cmd = cmd
        rate.sleep()

    return 0


if __name__ == "__main__":
    sys.exit(main())