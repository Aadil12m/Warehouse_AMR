#!/usr/bin/env python3
"""Web GUI backend bridge.

Starts/stops the Behavior Tree mission node and the OpenCV map-operations node
on dashboard button presses, and implements a REAL emergency stop:

  1. Kills spawned child processes by PROCESS GROUP, so children-of-children
     (explore_lite, ros2 launch) die too and nothing can send new goals.
  2. Cancels every goal on Nav2's navigate_to_pose / navigate_through_poses
     action servers. (Killing the BT process alone does NOT cancel an active
     Nav2 goal - the robot would keep driving.) Nav2 itself stays up, so the
     robot is usable again right after the e-stop.
  3. Publishes zero velocity continuously for a few seconds so any straggler
     publisher cannot re-start motion after the one-shot zero twist.
"""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from action_msgs.srv import CancelGoal


class GuiBridgeNode(Node):
    # Nav2 action servers whose goals make the robot move (explore_lite and the
    # Behavior Tree both drive through navigate_to_pose).
    NAV2_ACTIONS = ['navigate_to_pose', 'navigate_through_poses']
    ZERO_VEL_HOLD_S = 3.0

    def __init__(self):
        super().__init__('gui_bridge_node')

        self.mapping_sub = self.create_subscription(Empty, '/gui/trigger_mapping', self.mapping_callback, 10)
        self.orch_sub = self.create_subscription(Empty, '/gui/trigger_orchestration', self.orch_callback, 10)
        # The e-stop callback waits on Nav2 cancel responses, so it runs in a reentrant
        # group on a MultiThreadedExecutor (see main) where those responses can be
        # processed concurrently. Spinning from inside a callback is not allowed.
        self.estop_group = ReentrantCallbackGroup()
        self.estop_sub = self.create_subscription(Empty, '/gui/estop', self.estop_callback, 10,
                                                  callback_group=self.estop_group)

        self.zero_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mapping_process = None
        self.orch_process = None

        self.get_logger().info("GUI Bridge Node started. Listening for web commands...")

    # ------------------------------------------------------------------ utils
    def _spawn(self, argv):
        # Own process group so we can later kill the whole tree (ros2 run ->
        # robot_behavior -> system() -> explore_lite launch, etc).
        return subprocess.Popen(argv, preexec_fn=os.setsid)

    def _kill_group(self, proc, name):
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=5)
            self.get_logger().info(f"Stopped {name} (pid {proc.pid}) and its process group.")
        except (ProcessLookupError, PermissionError) as e:
            self.get_logger().warn(f"Could not stop {name}: {e}")

    def _cancel_all_nav_goals(self) -> bool:
        """Cancel every active Nav2 navigation goal (e-stop)."""
        ok_any = False
        for action in self.NAV2_ACTIONS:
            cli = self.create_client(CancelGoal, f'{action}/_action/cancel_goal',
                                     callback_group=self.estop_group)
            if not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"Nav2 action '{action}' unavailable; nothing to cancel.")
                self.destroy_client(cli)
                continue
            # Zero goal id + zero stamp = cancel ALL goals (action_msgs/srv/CancelGoal)
            future = cli.call_async(CancelGoal.Request())
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            res = future.result() if future.done() else None
            if res is not None:
                ok_any = True
                self.get_logger().info(
                    f"Cancelled {len(res.goals_canceling)} goal(s) on {action}.")
            else:
                self.get_logger().warn(f"Cancel request to {action} timed out.")
            self.destroy_client(cli)
        return ok_any

    def _publish_zero_velocity_hold(self):
        """Spam zero velocity for a short window so no stale stream can resume motion."""
        end_time = time.monotonic() + self.ZERO_VEL_HOLD_S
        zero = Twist()
        while time.monotonic() < end_time and rclpy.ok():
            self.zero_vel_pub.publish(zero)
            time.sleep(0.05)

    # --------------------------------------------------------------- triggers
    def mapping_callback(self, msg):
        if self.mapping_process is None or self.mapping_process.poll() is not None:
            self.get_logger().info("Starting Behavior Tree Mapping Node...")
            # use_sim_time must match the rest of the stack, or the explore_lite it
            # launches gets TF timeouts and never starts. Gazebo/Isaac publish /clock.
            sim = self.count_publishers('/clock') > 0
            self.get_logger().info(f"use_sim_time:={str(sim).lower()} (/clock {'found' if sim else 'not found'})")
            self.mapping_process = self._spawn(["ros2", "run", "robot_Behavior", "robot_behavior",
                                                "--ros-args", "-p", f"use_sim_time:={str(sim).lower()}"])
        else:
            self.get_logger().warn("Mapping is already running!")

    def orch_callback(self, msg):
        if self.orch_process is None or self.orch_process.poll() is not None:
            self.get_logger().info("Starting Map Operations (Orchestration) Node...")
            self.orch_process = self._spawn(["ros2", "run", "robot_navigation", "map_operation_node"])
        else:
            self.get_logger().warn("Orchestration is already running!")

    def estop_callback(self, msg):
        self.get_logger().error("E-STOP TRIGGERED: killing mission processes, cancelling Nav2 goals...")

        # 1. Kill mission processes AND their whole process groups, so nothing
        #    (BT, explore_lite) can send a fresh goal after we cancel.
        self._kill_group(self.mapping_process, "robot_behavior")
        self._kill_group(self.orch_process, "map_operation_node")
        self.mapping_process = None
        self.orch_process = None

        # 2. Abort whatever Nav2 is still executing (Nav2 itself stays up).
        self._cancel_all_nav_goals()

        # 3. Belt & braces in case anything else still holds /cmd_vel open.
        self._publish_zero_velocity_hold()

        self.get_logger().info("E-Stop complete. Robot idle and ready.")


def main(args=None):
    rclpy.init(args=args)
    node = GuiBridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Never leave the robot moving because the bridge died.
        try:
            zero = Twist()
            for _ in range(20):
                node.zero_vel_pub.publish(zero)
                time.sleep(0.02)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
