#!/usr/bin/env python3
"""Publish a circle or figure-eight as a native traj_utils/PolyTraj."""

import argparse
import math
import sys

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from traj_utils.msg import PolyTraj


def positive_float(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return parsed


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
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
            # The local +x axis is the vehicle's current forward direction.
            # This circle starts at its rear-most point, so its center is one
            # radius ahead and the complete circle stays in x >= 0.
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
        # A forward-facing Gerono figure eight. Its longitudinal range is
        # [0, 2 * radius], its crossing is radius ahead of the start, and its
        # lateral range is [-eight_width, eight_width].
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


def curve_arc_length(curve, theta_begin, theta_end):
    # Fixed Gauss-Legendre integration is deterministic and sufficiently
    # accurate for one short polynomial piece.
    nodes, weights = np.polynomial.legendre.leggauss(16)
    half_width = 0.5 * (theta_end - theta_begin)
    midpoint = 0.5 * (theta_end + theta_begin)
    length = 0.0
    for node, weight in zip(nodes, weights):
        _, derivative, _ = curve.evaluate(midpoint + half_width * node)
        length += weight * np.linalg.norm(derivative)
    return half_width * length


def constant_speed_state(curve, theta, speed):
    position, first, second = curve.evaluate(theta)
    derivative_norm = np.linalg.norm(first)
    if derivative_norm < 1.0e-9:
        raise ValueError("curve parameterization has a zero tangent")

    theta_dot = speed / derivative_norm
    theta_ddot = -speed * speed * np.dot(first, second) / derivative_norm ** 4
    velocity = first * theta_dot
    acceleration = second * theta_dot * theta_dot + first * theta_ddot
    return position, velocity, acceleration


def quintic_coefficients(position0, velocity0, acceleration0,
                         position1, velocity1, acceleration1, duration):
    """Return coefficients [t^5, ..., t^0] for a C2 quintic segment."""
    duration2 = duration * duration
    c0 = position0
    c1 = velocity0
    c2 = 0.5 * acceleration0
    c3 = (
        20.0 * (position1 - position0)
        - (8.0 * velocity1 + 12.0 * velocity0) * duration
        - (3.0 * acceleration0 - acceleration1) * duration2
    ) / (2.0 * duration ** 3)
    c4 = (
        30.0 * (position0 - position1)
        + (14.0 * velocity1 + 16.0 * velocity0) * duration
        + (3.0 * acceleration0 - 2.0 * acceleration1) * duration2
    ) / (2.0 * duration ** 4)
    c5 = (
        12.0 * (position1 - position0)
        - (6.0 * velocity1 + 6.0 * velocity0) * duration
        - (acceleration0 - acceleration1) * duration2
    ) / (2.0 * duration ** 5)
    return np.stack((c5, c4, c3, c2, c1, c0), axis=0)


def build_polytraj(args, odometry, start_time):
    current = odometry.pose.pose.position
    start_position = np.array(
        [current.x, current.y, current.z if args.z is None else args.z], dtype=float
    )
    if not np.isfinite(start_position).all():
        raise ValueError("odometry contains a non-finite position")

    yaw = quaternion_yaw(odometry.pose.pose.orientation)
    world_rotation = np.array(
        [[math.cos(yaw), -math.sin(yaw)],
         [math.sin(yaw), math.cos(yaw)]],
        dtype=float,
    )
    turn_sign = 1.0 if args.turn == "left" else -1.0
    eight_width = args.eight_width if args.eight_width is not None else args.radius
    curve = PlanarCurve(args.shape, args.radius, eight_width, turn_sign)

    piece_count = args.pieces_per_lap * args.laps
    theta_nodes = np.linspace(0.0, 2.0 * math.pi * args.laps, piece_count + 1)

    message = PolyTraj()
    message.drone_id = args.drone_id
    message.traj_id = args.traj_id
    message.start_time = start_time
    message.order = 5

    states = []
    for theta in theta_nodes:
        planar_position, planar_velocity, planar_acceleration = constant_speed_state(
            curve, theta, args.speed
        )
        position = start_position.copy()
        velocity = np.zeros(3)
        acceleration = np.zeros(3)
        position[:2] += world_rotation.dot(planar_position)
        velocity[:2] = world_rotation.dot(planar_velocity)
        acceleration[:2] = world_rotation.dot(planar_acceleration)
        states.append((position, velocity, acceleration))

    for piece in range(piece_count):
        duration = curve_arc_length(curve, theta_nodes[piece], theta_nodes[piece + 1])
        duration /= args.speed
        if not math.isfinite(duration) or duration <= 1.0e-4:
            raise ValueError("a generated polynomial piece is too short")
        message.duration.append(duration)

        coefficients = quintic_coefficients(
            *states[piece], *states[piece + 1], duration
        )
        message.coef_x.extend(coefficients[:, 0].tolist())
        message.coef_y.extend(coefficients[:, 1].tolist())
        message.coef_z.extend(coefficients[:, 2].tolist())

    return message, yaw


def make_argument_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Publish a C2 piecewise-quintic circle or figure-eight using the "
            "same traj_utils/PolyTraj format consumed by px4ctrl OM-MPC."
        )
    )
    parser.add_argument("--shape", choices=("circle", "eight"), default="circle")
    parser.add_argument("--radius", type=positive_float, default=1.0,
                        help="circle radius or figure-eight forward half-span [m]")
    parser.add_argument("--eight-width", type=positive_float, default=1.0,
                        help="figure-eight lateral half-width [m] (default: radius)")
    parser.add_argument("--speed", type=positive_float, default=0.5,
                        help="approximately constant tracking speed [m/s]")
    parser.add_argument("--laps", type=positive_int, default=10)
    parser.add_argument("--pieces-per-lap", type=positive_int, default=32,
                        help="number of quintic pieces per circle/eight cycle")
    parser.add_argument("--turn", choices=("left", "right"), default="left",
                        help="circle initially moves toward vehicle left or right")
    parser.add_argument("--z", type=float, default=1.2,
                        help="fixed world-frame altitude [m] (default: current z)")
    parser.add_argument("--odom-topic", default="/lidar_slam/imu_propagate")
    parser.add_argument("--trajectory-topic", default="/drone_0_planning/trajectory")
    parser.add_argument("--heartbeat-topic", default="/drone_0_traj_server/heartbeat")
    parser.add_argument("--heartbeat-rate", type=positive_float, default=20.0)
    parser.add_argument("--no-heartbeat", action="store_true",
                        help="do not emulate the planner heartbeat")
    parser.add_argument("--start-delay", type=positive_float, default=0.5,
                        help="delay between publication and trajectory start [s]")
    parser.add_argument("--odom-timeout", type=positive_float, default=5.0)
    parser.add_argument("--drone-id", type=int, default=0)
    parser.add_argument("--traj-id", type=positive_int, default=1)
    return parser


def main():
    args = make_argument_parser().parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node("polytraj_shape_publisher", anonymous=True)

    trajectory_publisher = rospy.Publisher(
        args.trajectory_topic, PolyTraj, queue_size=1, latch=True
    )
    heartbeat_publisher = rospy.Publisher(args.heartbeat_topic, Empty, queue_size=1)

    rospy.loginfo("Waiting up to %.1f s for odometry on %s", args.odom_timeout,
                  args.odom_topic)
    try:
        odometry = rospy.wait_for_message(
            args.odom_topic, Odometry, timeout=args.odom_timeout
        )
    except rospy.ROSException as error:
        rospy.logfatal("No odometry received: %s", error)
        return 1

    try:
        start_time = rospy.Time.now() + rospy.Duration(args.start_delay)
        message, yaw = build_polytraj(args, odometry, start_time)
    except ValueError as error:
        rospy.logfatal("Cannot generate trajectory: %s", error)
        return 1

    if not args.no_heartbeat:
        heartbeat_publisher.publish(Empty())
    trajectory_publisher.publish(message)

    total_duration = sum(message.duration)
    rospy.loginfo(
        "Published %s PolyTraj id=%d: %d pieces, %.3f s, start=(%.3f, %.3f, %.3f), "
        "forward_yaw=%.3f rad, speed=%.3f m/s",
        args.shape, message.traj_id, len(message.duration), total_duration,
        message.coef_x[5], message.coef_y[5], message.coef_z[5], yaw, args.speed,
    )

    if args.no_heartbeat:
        rospy.sleep(0.2)
        return 0

    # Keep emulating the planner for the complete scheduled trajectory. The
    # extra second lets px4ctrl observe trajectory completion before heartbeat
    # publication stops.
    stop_time = message.start_time + rospy.Duration(total_duration + 1.0)
    rate = rospy.Rate(args.heartbeat_rate)
    while not rospy.is_shutdown() and rospy.Time.now() <= stop_time:
        heartbeat_publisher.publish(Empty())
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
