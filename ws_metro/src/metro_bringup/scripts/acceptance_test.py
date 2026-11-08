#!/usr/bin/env python3
"""End-to-end acceptance scenario for the metro PSD interlock simulation.

It exercises the public interface only (topics/services from the contract) and
checks:

  R1  interlock disabled -> open request is refused and doors stay closed
  R2  train outside the stop window -> open refused, TRAIN_NOT_IN_STOP_WINDOW
  R3  doors not locked -> train command forced to zero, PSD_NOT_LOCKED
  R4  open timeout branch (fault injected with a negative timeout while the
      train is stopped); nominal path also asserts state changes within 3 s
  R5  train moving (> 0.1 m/s) with doors open -> EMERGENCY_BRAKE_PSD_OPEN

The script uses wall time for its own test timeouts (it is a test harness, not
a flight node) and uses sim time only through the /clock subscription.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


FAULT_R2 = "TRAIN_NOT_IN_STOP_WINDOW"
FAULT_R3 = "PSD_NOT_LOCKED"
FAULT_R4 = "PSD_OPEN_TIMEOUT"
FAULT_R5 = "EMERGENCY_BRAKE_PSD_OPEN"


class AcceptanceTester(Node):
    def __init__(self):
        super().__init__(
            "metro_acceptance_tester",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )

        self.odom = None
        self.psd_state = None
        self.train_cmd = None
        self.faults = []
        self.last_fault = ""

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(Odometry, "/metro/train/odom", self._on_odom, sensor_qos)
        self.create_subscription(Bool, "/metro/psd/state", self._on_psd, reliable_qos)
        self.create_subscription(Twist, "/metro/train/command", self._on_cmd, reliable_qos)
        self.create_subscription(String, "/metro/interlock/fault", self._on_fault, reliable_qos)

        self.enable_client = self.create_client(SetBool, "/metro/interlock/enable")
        self.psd_client = self.create_client(SetBool, "/metro/psd/command")
        self.param_client = self.create_client(SetParameters, "/interlock_node/set_parameters")

        self.failures = []
        self.checks = 0

    # ------------------------------------------------------------------ callbacks
    def _on_odom(self, msg):
        self.odom = msg

    def _on_psd(self, msg):
        self.psd_state = msg.data

    def _on_cmd(self, msg):
        self.train_cmd = msg

    def _on_fault(self, msg):
        code = msg.data
        self.last_fault = code
        if code:
            self.faults.append(code)

    # ------------------------------------------------------------------ utilities
    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, description, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            try:
                if predicate():
                    return True
            except Exception:  # noqa: BLE001 - predicate may race with messages
                pass
        self.fail(f"timeout waiting for {description}")
        return False

    def wait_for_service(self, client, name, timeout_s=15.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.fail(f"service not available: {name}")
        return False

    def call_bool(self, client, value, timeout_s=5.0):
        request = SetBool.Request()
        request.data = bool(value)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if future.done():
                return future.result()
        self.fail(f"service call timed out (value={value})")
        return None

    def set_parameters(self, pairs):
        request = SetParameters.Request()
        for name, value in pairs:
            parameter = ParameterMsg()
            parameter.name = name
            parameter.value = ParameterValue()
            if isinstance(value, bool):
                parameter.value.type = ParameterType.PARAMETER_BOOL
                parameter.value.bool_value = value
            elif isinstance(value, float):
                parameter.value.type = ParameterType.PARAMETER_DOUBLE
                parameter.value.double_value = value
            elif isinstance(value, int):
                parameter.value.type = ParameterType.PARAMETER_INTEGER
                parameter.value.integer_value = value
            else:
                parameter.value.type = ParameterType.PARAMETER_STRING
                parameter.value.string_value = str(value)
            request.parameters.append(parameter)

        future = self.param_client.call_async(request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if future.done():
                results = future.result().results
                return all(item.successful for item in results)
        self.fail("set_parameters call timed out")
        return False

    def ok(self, condition, message):
        self.checks += 1
        if condition:
            self.get_logger().info(f"PASS: {message}")
        else:
            self.fail(message)

    def fail(self, message):
        self.get_logger().error(f"FAIL: {message}")
        self.failures.append(message)

    def clear_faults(self):
        self.faults = []
        self.last_fault = ""

    def wait_for_fault(self, code, timeout_s=3.0):
        return self.wait_for(
            f"fault {code}", lambda: code in self.faults, timeout_s
        )

    def wait_for_fault_clear(self, timeout_s=4.0):
        return self.wait_for(
            "fault clear", lambda: self.last_fault == "", timeout_s
        )

    # ------------------------------------------------------------------ checks
    def run(self):
        self.get_logger().info("waiting for /clock and services ...")
        if not self.wait_for(
            "sim clock", lambda: self.get_clock().now().nanoseconds > 0, 30.0
        ):
            return False
        for client, name in (
            (self.enable_client, "/metro/interlock/enable"),
            (self.psd_client, "/metro/psd/command"),
            (self.param_client, "/interlock_node/set_parameters"),
        ):
            if not self.wait_for_service(client, name):
                return False

        if not self.wait_for(
            "initial PSD state true",
            lambda: self.psd_state is True,
            15.0,
        ):
            return False
        self.ok(True, "PSD reports locked/closed at startup")

        # Force deterministic manual mode while the train is far from the station.
        self.set_parameters([("auto_run", False), ("manual_speed_mps", 0.0)])
        if not self.wait_for(
            "train outside stop window",
            lambda: self.odom is not None and self.odom.pose.pose.position.x < -1.0,
            15.0,
        ):
            return False

        # ---- R1: disabled interlock refuses opening, no fault -----------------
        self.ok(
            self.call_bool(self.enable_client, False).success,
            "interlock disable service accepted",
        )
        self.wait_for_fault_clear()
        self.clear_faults()
        response = self.call_bool(self.psd_client, True)
        self.ok(response is not None and not response.success, "R1: open refused while disabled")
        self.spin_for(1.5)
        self.ok(self.psd_state is True, "R1: PSD stayed closed")
        self.ok(not self.faults, f"R1: no fault raised (got {self.faults})")

        # ---- R2: disabled -> enable, train outside window -> refuse -----------
        self.ok(
            self.call_bool(self.enable_client, True).success,
            "interlock enable service accepted",
        )
        self.clear_faults()
        response = self.call_bool(self.psd_client, True)
        self.ok(response is not None and not response.success, "R2: open refused outside window")
        self.wait_for_fault(FAULT_R2)
        self.ok(FAULT_R2 in self.faults, f"R2: fault {FAULT_R2} published")
        self.ok(self.psd_state is True, "R2: PSD stayed closed")

        # ---- nominal automatic stop ------------------------------------------
        self.set_parameters([("auto_run", True)])
        if not self.wait_for(
            "train stopped in window",
            lambda: self.odom is not None
            and abs(self.odom.pose.pose.position.x) <= 0.5
            and abs(self.odom.twist.twist.linear.x) < 0.05,
            90.0,
        ):
            return False
        self.ok(True, "speed curve stopped the train inside +/-0.5 m")
        self.spin_for(0.5)
        self.ok(
            self.train_cmd is not None and abs(self.train_cmd.linear.x) < 0.05,
            "train command is zero at standstill",
        )

        # ---- R3 + nominal open within 3 s ------------------------------------
        self.clear_faults()
        response = self.call_bool(self.psd_client, True)
        self.ok(response is not None and response.success, "open accepted in window")
        if not self.wait_for("PSD open", lambda: self.psd_state is False, 3.0):
            return False
        self.ok(True, "R4 nominal: PSD state became false well within 3 s")
        self.wait_for_fault(FAULT_R3)
        self.ok(FAULT_R3 in self.faults, f"R3: fault {FAULT_R3} published")
        self.spin_for(0.3)
        self.ok(
            self.train_cmd is not None and abs(self.train_cmd.linear.x) < 1e-6,
            "R3: train command forced to zero while doors open",
        )

        # ---- close and wait until locked -------------------------------------
        self.set_parameters([("auto_run", False), ("manual_speed_mps", 0.0)])
        self.ok(
            self.call_bool(self.psd_client, False).success,
            "close command accepted",
        )
        if not self.wait_for("PSD closed", lambda: self.psd_state is True, 8.0):
            return False
        self.ok(True, "PSD locked after close")
        self.wait_for_fault_clear()

        # ---- R4 timeout branch (fault injection: negative timeout) -----------
        self.set_parameters([("auto_run", False), ("manual_speed_mps", 0.0)])
        self.set_parameters([("psd_open_timeout_s", -1.0)])
        self.clear_faults()
        self.call_bool(self.psd_client, True)
        self.wait_for_fault(FAULT_R4, 4.0)
        self.ok(FAULT_R4 in self.faults, f"R4: fault {FAULT_R4} published on timeout")
        self.set_parameters([("psd_open_timeout_s", 3.0)])
        self.call_bool(self.psd_client, False)
        self.wait_for("PSD closed after R4 test", lambda: self.psd_state is True, 8.0)

        # ---- R5: moving train with doors open --------------------------------
        self.set_parameters([("auto_run", False), ("manual_speed_mps", 0.6)])
        if not self.wait_for(
            "train moving inside window",
            lambda: self.odom is not None
            and abs(self.odom.pose.pose.position.x) <= 0.4
            and self.odom.twist.twist.linear.x > 0.15,
            10.0,
        ):
            return False
        self.clear_faults()
        response = self.call_bool(self.psd_client, True)
        self.ok(response is not None and response.success, "R5: open accepted while moving in window")
        self.wait_for_fault(FAULT_R5, 4.0)
        self.ok(FAULT_R5 in self.faults, f"R5: fault {FAULT_R5} published")
        self.spin_for(0.5)
        self.ok(
            self.train_cmd is not None and abs(self.train_cmd.linear.x) < 1e-6,
            "R5: train command forced to zero by emergency brake",
        )

        # cleanup
        self.set_parameters([("manual_speed_mps", 0.0)])
        self.call_bool(self.psd_client, False)
        self.spin_for(1.0)
        return True


def main():
    rclpy.init()
    node = AcceptanceTester()
    success = False
    try:
        success = node.run()
    except Exception as exc:  # noqa: BLE001 - test harness must report failures
        node.get_logger().error(f"acceptance scenario crashed: {exc}")
        success = False
    finally:
        total = node.checks
        failures = len(node.failures)
        node.get_logger().info(f"acceptance summary: {total - failures}/{total} checks passed")
        for failure in node.failures:
            node.get_logger().error(f"  - {failure}")
        node.destroy_node()
        rclpy.shutdown()
    print(f"ACCEPTANCE_RESULT={'PASS' if success and not node.failures else 'FAIL'}")
    sys.exit(0 if success and not node.failures else 1)


if __name__ == "__main__":
    main()
