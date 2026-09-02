"""
Integration and E2E Tests for McuBridgeNode with VirtualMCU
Verifies ROS 2 topics, serial communication, telemetry conversion, command forwarding, and fault recovery.
"""

import time
import pytest
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState

from humanoid_mobile_mcu_ros.msg import (
    RobotTelemetry,
    BmsTelemetry,
    PowerCmd,
    LedCmd,
    SignalCmd,
    BmsCmd
)

from humanoid_mobile_mcu_ros.mcu_bridge_node import McuBridgeNode
from humanoid_mobile_mcu_ros.protocol import (
    MSG_ID_CMD_REQUEST_INFO,
    MSG_ID_CMD_POWER_CTRL,
    MSG_ID_CMD_LED_CTRL,
    MSG_ID_CMD_SIGNAL_CTRL,
    MSG_ID_CMD_BMS_CTRL
)

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from virtual_mcu import VirtualMCU


class TestMcuBridgeNodeE2E:
    """E2E Test Suite connecting McuBridgeNode to VirtualMCU over Linux PTY"""

    @classmethod
    def setup_class(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def teardown_class(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setup_method(self):
        # 1. Start Virtual MCU Simulator
        self.vmcu = VirtualMCU(start_telemetry=True)
        time.sleep(0.1)

        # 2. Start McuBridgeNode configured to use the virtual slave PTY port
        # Note: Set reconnect_interval_sec to 0.5 for fast test execution
        rclpy.init(args=None) if not rclpy.ok() else None
        
        # Override parameters
        self.node = McuBridgeNode()
        # Change serial port to virtual PTY slave
        self.node.port = self.vmcu.slave_name
        self.node.reconnect_interval = 0.5

        # Helper Test Node to publish/subscribe
        self.test_helper = Node("test_helper_node")

        # Start executor thread
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.test_helper)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # Wait for serial connection to establish
        time.sleep(0.8)

    def teardown_method(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.test_helper.destroy_node()
        self.vmcu.close()
        time.sleep(0.2)

    def test_01_handshake_system_info(self):
        """Verify McuBridgeNode requests System Info upon connection"""
        cmds = self.vmcu.get_received_commands()
        req_info_cmds = [c for c in cmds if c[0] == MSG_ID_CMD_REQUEST_INFO]
        assert len(req_info_cmds) >= 1, "McuBridgeNode should request System Info upon connection"

    def test_02_robot_telemetry_subscription(self):
        """Verify /mcu/robot_telemetry and /mcu/emergency_status are published correctly"""
        received_msgs: List[RobotTelemetry] = []
        received_ems: List[Bool] = []

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        sub_telem = self.test_helper.create_subscription(
            RobotTelemetry, "/mcu/robot_telemetry",
            lambda msg: received_msgs.append(msg), sensor_qos
        )
        sub_ems = self.test_helper.create_subscription(
            Bool, "/mcu/emergency_status",
            lambda msg: received_ems.append(msg), 10
        )

        # Wait for telemetry to be received
        for _ in range(30):
            if len(received_msgs) >= 3 and len(received_ems) >= 3:
                break
            time.sleep(0.1)

        assert len(received_msgs) >= 1, "Should receive RobotTelemetry messages at 20Hz"
        assert len(received_ems) >= 1, "Should receive emergency_status messages"

        latest = received_msgs[-1]
        assert latest.header.frame_id == "mcu_link"
        assert latest.start_btn is True       # Configured in VirtualMCU default
        assert latest.nav_pc_live is True
        assert latest.humanoid_pc_live is True
        assert latest.ai_pc_live is True
        assert latest.pwr_hold is True
        assert latest.volt24v_1 is True
        assert latest.volt24v_2 is True
        assert latest.safety_ems is False

    def test_03_bms_telemetry_and_battery_state(self):
        """Verify /mcu/bms_telemetry and sensor_msgs/BatteryState are published correctly"""
        bms_msgs: List[BmsTelemetry] = []
        batt_msgs: List[BatteryState] = []

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        sub_bms = self.test_helper.create_subscription(
            BmsTelemetry, "/mcu/bms_telemetry",
            lambda msg: bms_msgs.append(msg), sensor_qos
        )
        sub_batt = self.test_helper.create_subscription(
            BatteryState, "/mcu/battery_state",
            lambda msg: batt_msgs.append(msg), sensor_qos
        )

        # Wait for 1Hz BMS message
        for _ in range(30):
            if len(bms_msgs) >= 1 and len(batt_msgs) >= 1:
                break
            time.sleep(0.1)

        assert len(bms_msgs) >= 1, "Should receive BmsTelemetry message"
        assert len(batt_msgs) >= 1, "Should receive BatteryState message"

        bms = bms_msgs[-1]
        assert pytest.approx(bms.voltage, 0.1) == 48.5
        assert pytest.approx(bms.current, 0.1) == -3.2
        assert pytest.approx(bms.soc, 0.1) == 88.0
        assert bms.is_valid is True

        batt = batt_msgs[-1]
        assert pytest.approx(batt.voltage, 0.1) == 48.5
        assert pytest.approx(batt.percentage, 0.01) == 0.88
        assert batt.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

    def test_04_cmd_power_forwarding(self):
        """Verify publishing /mcu/cmd_power sends correct binary packet to MCU"""
        pub = self.test_helper.create_publisher(PowerCmd, "/mcu/cmd_power", 10)
        time.sleep(0.2)

        self.vmcu.clear_received_commands()

        msg = PowerCmd()
        msg.target_device = PowerCmd.TARGET_HUMANOID_PC
        msg.action = PowerCmd.ACTION_PULSE
        pub.publish(msg)

        # Wait for serial transmission
        time.sleep(0.3)

        cmds = self.vmcu.get_received_commands()
        power_cmds = [c for c in cmds if c[0] == MSG_ID_CMD_POWER_CTRL]
        assert len(power_cmds) >= 1, "VirtualMCU should receive MSG_ID_CMD_POWER_CTRL"
        assert power_cmds[-1][1] == bytes([PowerCmd.TARGET_HUMANOID_PC, PowerCmd.ACTION_PULSE])

    def test_05_cmd_led_forwarding(self):
        """Verify publishing /mcu/cmd_led sends correct binary packet to MCU"""
        pub = self.test_helper.create_publisher(LedCmd, "/mcu/cmd_led", 10)
        time.sleep(0.2)

        self.vmcu.clear_received_commands()

        msg = LedCmd()
        msg.mode = LedCmd.FULL_GREEN # 7
        msg.r = 0
        msg.g = 255
        msg.b = 0
        msg.brightness = 180
        pub.publish(msg)

        time.sleep(0.3)

        cmds = self.vmcu.get_received_commands()
        led_cmds = [c for c in cmds if c[0] == MSG_ID_CMD_LED_CTRL]
        assert len(led_cmds) >= 1, "VirtualMCU should receive MSG_ID_CMD_LED_CTRL"
        assert led_cmds[-1][1] == bytes([7, 0, 255, 0, 180])

    def test_06_cmd_signal_forwarding(self):
        """Verify publishing /mcu/cmd_signal sends correct binary packet to MCU"""
        pub = self.test_helper.create_publisher(SignalCmd, "/mcu/cmd_signal", 10)
        time.sleep(0.2)

        self.vmcu.clear_received_commands()

        msg = SignalCmd()
        msg.lamp_red = SignalCmd.STATE_ON
        msg.lamp_grn = SignalCmd.STATE_OFF
        msg.lamp_yel = SignalCmd.STATE_NO_CHANGE
        msg.buzzer = SignalCmd.STATE_ON
        msg.tm_remote = SignalCmd.STATE_OFF
        pub.publish(msg)

        time.sleep(0.3)

        cmds = self.vmcu.get_received_commands()
        sig_cmds = [c for c in cmds if c[0] == MSG_ID_CMD_SIGNAL_CTRL]
        assert len(sig_cmds) >= 1, "VirtualMCU should receive MSG_ID_CMD_SIGNAL_CTRL"
        assert sig_cmds[-1][1] == bytes([1, 0, 255, 1, 0])

    def test_07_cmd_bms_forwarding(self):
        """Verify publishing /mcu/cmd_bms sends correct binary packet to MCU"""
        pub = self.test_helper.create_publisher(BmsCmd, "/mcu/cmd_bms", 10)
        time.sleep(0.2)

        self.vmcu.clear_received_commands()

        msg = BmsCmd()
        msg.bms_reset_pulse = 1
        msg.chg_enable = 1
        pub.publish(msg)

        time.sleep(0.3)

        cmds = self.vmcu.get_received_commands()
        bms_cmds = [c for c in cmds if c[0] == MSG_ID_CMD_BMS_CTRL]
        assert len(bms_cmds) >= 1, "VirtualMCU should receive MSG_ID_CMD_BMS_CTRL"
        assert bms_cmds[-1][1] == bytes([1, 1])

    def test_08_noise_recovery(self):
        """Verify bridge node continues receiving telemetry normally after serial noise injection"""
        received_msgs: List[RobotTelemetry] = []
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        sub = self.test_helper.create_subscription(
            RobotTelemetry, "/mcu/robot_telemetry",
            lambda msg: received_msgs.append(msg), sensor_qos
        )

        # Inject noise bytes into serial stream
        self.vmcu.inject_noise(b"\xFF\xAA\x12\x34\xFE\xDD\x00\x55\xAA")
        time.sleep(0.5)

        initial_count = len(received_msgs)
        time.sleep(0.4) # Wait for ~8 telemetry cycles (20Hz)

        assert len(received_msgs) > initial_count, "Telemetry reception should continue uninterrupted after noise"
