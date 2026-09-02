"""
ROS 2 MCU Serial Bridge Node for Mobile Humanoid
"""

import sys
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from std_msgs.msg import Header, Bool
from sensor_msgs.msg import BatteryState

from humanoid_mobile_mcu_ros.msg import (
    RobotTelemetry,
    BmsTelemetry,
    PowerCmd,
    LedCmd,
    SignalCmd,
    BmsCmd
)

from .protocol import (
    MSG_ID_ROBOT_TELEMETRY,
    MSG_ID_BMS_TELEMETRY,
    MSG_ID_SYSTEM_INFO,
    MSG_ID_ACK_NACK,
    BinaryPacketParser,
    build_power_cmd,
    build_led_cmd,
    build_signal_cmd,
    build_bms_cmd,
    build_request_info_cmd,
    parse_robot_telemetry,
    parse_bms_telemetry,
    parse_ack,
    parse_system_info
)


class McuBridgeNode(Node):
    def __init__(self):
        super().__init__("mcu_bridge_node")

        # Declare parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 57600)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("frame_id", "mcu_link")
        self.declare_parameter("publish_battery_state", True)

        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.reconnect_interval = self.get_parameter("reconnect_interval_sec").get_parameter_value().double_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.publish_battery_state = self.get_parameter("publish_battery_state").get_parameter_value().bool_value

        self.get_logger().info(
            f"Starting MCU Serial Bridge Node on port '{self.port}' at {self.baud_rate} baud."
        )

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.pub_robot_telemetry = self.create_publisher(RobotTelemetry, "/mcu/robot_telemetry", sensor_qos)
        self.pub_bms_telemetry = self.create_publisher(BmsTelemetry, "/mcu/bms_telemetry", sensor_qos)
        self.pub_emergency_status = self.create_publisher(Bool, "/mcu/emergency_status", cmd_qos)
        if self.publish_battery_state:
            self.pub_battery_state = self.create_publisher(BatteryState, "/mcu/battery_state", sensor_qos)

        # Subscribers
        self.sub_cmd_power = self.create_subscription(
            PowerCmd, "/mcu/cmd_power", self.cb_cmd_power, cmd_qos
        )
        self.sub_cmd_led = self.create_subscription(
            LedCmd, "/mcu/cmd_led", self.cb_cmd_led, cmd_qos
        )
        self.sub_cmd_signal = self.create_subscription(
            SignalCmd, "/mcu/cmd_signal", self.cb_cmd_signal, cmd_qos
        )
        self.sub_cmd_bms = self.create_subscription(
            BmsCmd, "/mcu/cmd_bms", self.cb_cmd_bms, cmd_qos
        )

        # Serial communication internals
        self.ser: Optional[serial.Serial] = None
        self.tx_lock = threading.Lock()
        self.running = True
        self.parser = BinaryPacketParser(self.on_packet_received)

        # Launch background serial communication thread
        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()

    # =========================================================================
    # Serial Worker & Reconnect Loop
    # =========================================================================
    def _serial_worker(self):
        while self.running and rclpy.ok():
            try:
                if self.ser is None or not self.ser.is_open:
                    self.get_logger().info(f"Connecting to serial port '{self.port}'...")
                    self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.1)
                    self.get_logger().info(f"Successfully connected to '{self.port}'.")

                    # Request System Info after connection
                    time.sleep(0.5)
                    self._send_raw_packet(build_request_info_cmd())

                # Read available incoming data
                data = self.ser.read(64)
                if data:
                    for b in data:
                        self.parser.parse_byte(b)

            except (serial.SerialException, OSError, TypeError) as e:
                if not self.running:
                    break
                self.get_logger().warn(
                    f"Serial error on port '{self.port}': {e}. Reconnecting in {self.reconnect_interval:.1f}s..."
                )
                if self.ser is not None:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None

                time.sleep(self.reconnect_interval)

            except Exception as e:
                self.get_logger().error(f"Unexpected error in serial worker: {e}")
                time.sleep(self.reconnect_interval)

    def _send_raw_packet(self, packet: bytes) -> bool:
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("Cannot send packet: Serial port is not open.")
            return False
        with self.tx_lock:
            try:
                self.ser.write(packet)
                return True
            except Exception as e:
                self.get_logger().error(f"Failed to write packet to serial port: {e}")
                return False

    # =========================================================================
    # Incoming Packet Dispatcher
    # =========================================================================
    def on_packet_received(self, msg_id: int, payload: bytes):
        now_stamp = self.get_clock().now().to_msg()

        if msg_id == MSG_ID_ROBOT_TELEMETRY:
            data = parse_robot_telemetry(payload)
            if data is None:
                return

            msg = RobotTelemetry()
            msg.header.stamp = now_stamp
            msg.header.frame_id = self.frame_id

            msg.pwr_btn = data["pwr_btn"]
            msg.start_btn = data["start_btn"]
            msg.stop_btn = data["stop_btn"]
            msg.manu_sw = data["manu_sw"]

            msg.nav_pc_live = data["nav_pc_live"]
            msg.humanoid_pc_live = data["humanoid_pc_live"]
            msg.ai_pc_live = data["ai_pc_live"]

            msg.safety_ems = data["safety_ems"]
            msg.manu_chg_dock = data["manu_chg_dock"]
            msg.is_charging = data["is_charging"]

            msg.pwr_hold = data["pwr_hold"]
            msg.volt24v_1 = data["volt24v_1"]
            msg.volt24v_2 = data["volt24v_2"]
            msg.chg_on_enable = data["chg_on_enable"]
            msg.lamp_red = data["lamp_red"]
            msg.lamp_grn = data["lamp_grn"]
            msg.lamp_yel = data["lamp_yel"]
            msg.buzzer = data["buzzer"]

            msg.chg_current_adc = data["chg_current_adc"]
            msg.led_current_mode = data["led_current_mode"]
            msg.uptime_sec = data["uptime_sec"]

            self.pub_robot_telemetry.publish(msg)

            # Publish Emergency Status separately for quick monitoring
            ems_msg = Bool()
            ems_msg.data = data["safety_ems"]
            self.pub_emergency_status.publish(ems_msg)

        elif msg_id == MSG_ID_BMS_TELEMETRY:
            data = parse_bms_telemetry(payload)
            if data is None:
                return

            msg = BmsTelemetry()
            msg.header.stamp = now_stamp
            msg.header.frame_id = self.frame_id
            msg.voltage = data["voltage"]
            msg.current = data["current"]
            msg.soc = data["soc"]
            msg.soh = data["soh"]
            msg.temp = data["temp"]
            msg.is_valid = data["is_valid"]
            self.pub_bms_telemetry.publish(msg)

            # Publish standard ROS 2 BatteryState message
            if self.publish_battery_state:
                batt = BatteryState()
                batt.header.stamp = now_stamp
                batt.header.frame_id = self.frame_id
                batt.voltage = data["voltage"]
                batt.current = data["current"]
                batt.percentage = data["soc"] / 100.0  # 0.0 ~ 1.0 standard

                if data["is_valid"]:
                    if data["current"] > 0.1:
                        batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                    elif data["current"] < -0.1:
                        batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                    else:
                        batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
                    batt.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
                else:
                    batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                    batt.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN

                self.pub_battery_state.publish(batt)

        elif msg_id == MSG_ID_SYSTEM_INFO:
            data = parse_system_info(payload)
            if data:
                self.get_logger().info(
                    f"MCU System Info: SW='{data['sw_ver']}', HW='{data['hw_ver']}'"
                )

        elif msg_id == MSG_ID_ACK_NACK:
            data = parse_ack(payload)
            if data:
                self.get_logger().debug(
                    f"MCU ACK received: TargetMsgID=0x{data['target_msg_id']:02X}, Status={data['status_str']}"
                )

    # =========================================================================
    # ROS 2 Command Callbacks -> Serial Send
    # =========================================================================
    def cb_cmd_power(self, msg: PowerCmd):
        self.get_logger().info(
            f"Received PowerCmd: Target={msg.target_device}, Action={msg.action}"
        )
        packet = build_power_cmd(msg.target_device, msg.action)
        self._send_raw_packet(packet)

    def cb_cmd_led(self, msg: LedCmd):
        brightness = msg.brightness if msg.brightness > 0 else 200
        self.get_logger().info(
            f"Received LedCmd: Mode={msg.mode}, RGB=({msg.r},{msg.g},{msg.b}), Brightness={brightness}"
        )
        packet = build_led_cmd(msg.mode, msg.r, msg.g, msg.b, brightness)
        self._send_raw_packet(packet)

    def cb_cmd_signal(self, msg: SignalCmd):
        self.get_logger().info(
            f"Received SignalCmd: Tower[R={msg.lamp_red}, G={msg.lamp_grn}, Y={msg.lamp_yel}], Buzzer={msg.buzzer}, TM={msg.tm_remote}"
        )
        packet = build_signal_cmd(msg.lamp_red, msg.lamp_grn, msg.lamp_yel, msg.buzzer, msg.tm_remote)
        self._send_raw_packet(packet)

    def cb_cmd_bms(self, msg: BmsCmd):
        self.get_logger().info(
            f"Received BmsCmd: ResetPulse={msg.bms_reset_pulse}, ChgEnable={msg.chg_enable}"
        )
        packet = build_bms_cmd(msg.bms_reset_pulse, msg.chg_enable)
        self._send_raw_packet(packet)

    def destroy_node(self):
        self.running = False
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = McuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
