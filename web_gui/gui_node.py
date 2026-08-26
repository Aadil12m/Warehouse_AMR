#!/usr/bin/env python3
"""Web GUI backend bridge.

Starts/stops the Behavior Tree mission node and the OpenCV map-operations node
on dashboard button presses, and implements a REAL emergency stop:

  1. Deactivates Nav2's bt_navigator + controller_server via the lifecycle
     manager -> any goal in flight is aborted and /cmd_vel stops being
     published. (Killing the BT process alone does NOT cancel an active Nav2
     goal - the robot would keep driving.)
  2. Kills spawned child processes by PROCESS GROUP, so children-of-children
     (explore_lite, ros2 launch) die too. Previously `pkill -f` missed the
     explore node spawned via system() inside robot_behavior.
  3. Publishes zero velocity continuously for a few seconds so any straggler
     publisher cannot re-start motion after the one-shot zero twist.
"""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class GuiBridgeNode(Node):
    # Nav2 nodes that actually emit /cmd_vel. Deactivating them halts motion;
    # they are re-activated afterwards so the stack stays usable post-estop.
    LIFECYCLE_NODES = ['controller_server', 'bt_navigator']
    MANAGED_BY = 'nav2_manager'  # default lifecycle_manager name from nav2_bringup
    ZERO_VEL_HOLD_S = 3.0

    def __init__(self):
        super().__init__('gui_bridge_node')

        self.mapping_sub = self.create_subscription(Empty, '/gui/trigger_mapping', self.mapping_callback, 10)
        self.orch_sub = self.create_subscription(Empty, '/gui/trigger_orchestration', self.orch_callback, 10)
        self.estop_sub = self.create_subscription(Empty, '/gui/estop', self.estop_callback, 10)

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

    def _set_nav_lifecycle(self, activate: bool) -> bool:
        """Deactivate (estop) / reactivate Nav2 motion nodes."""
        cli = self.create_client(ChangeState, f'{self.MANAGED_BY}/change_state')
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"Lifecycle manager '{self.MANAGED_BY}' unavailable; cannot "
                f"{'resume' if activate else 'halt'} Nav2 programmatically.")
            return False

        transition = Transition.TRANSITION_DEACTIVATE if not activate else Transition.TRANSITION_ACTIVATE
        ok_any = False
        for node_name in self.LIFECYCLE_NODES:
            req = ChangeState.Request()
            req.transition.id = transition
            # nav2 lifecycle manager routes node_label through transition.label
            req.transition.label = node_name
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is not None and future.result().success:
                ok_any = True
            else:
                self.get_logger().warn(f"Lifecycle change for {node_name} failed.")
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
            self.mapping_process = self._spawn(["ros2", "run", "robot_Behavior", "robot_behavior"])
        else:
            self.get_logger().warn("Mapping is already running!")

    def orch_callback(self, msg):
        if self.orch_process is None or self.orch_process.poll() is not None:
            self.get_logger().info("Starting Map Operations (Orchestration) Node...")
            self.orch_process = self._spawn(["ros2", "run", "robot_navigation", "map_operation_node"])
        else:
            self.get_logger().warn("Orchestration is already running!")

    def estop_callback(self, msg):
        self.get_logger().error("E-STOP TRIGGERED: halting Nav2, killing mission processes...")

        # 1. Stop the motion pipeline at the source (aborts active goals).
        self._set_nav_lifecycle(activate=False)

        # 2. Kill mission processes AND their whole process groups.
        self._kill_group(self.mapping_process, "robot_behavior")
        self._kill_group(self.orch_process, "map_operation_node")
        self.mapping_process = None
        self.orch_process = None

        # Belt & braces in case anything else still holds /cmd_vel open.
        self._publish_zero_velocity_hold()

        # 3. Bring Nav2 back so the operator can keep using the robot.
        if self._set_nav_lifecycle(activate=True):
            self.get_logger().info("Nav2 re-activated. Robot idle and ready.")

        self.get_logger().info("E-Stop complete.")


def main(args=None):
    rclpy.init(args=args)
    node = GuiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GUI Bridge Node.")
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
