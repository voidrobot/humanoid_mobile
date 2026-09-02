"""
Virtual MCU Simulator for Mobile Humanoid
Simulates ATmega2560 MCU Firmware (mobile/humanoid_mobile_mcu) using Linux PTY (Pseudo-Terminal)
"""

import os
import pty
import tty
import time
import struct
import threading
from typing import Optional, List, Tuple, Dict, Any

from humanoid_mobile_mcu_ros.protocol import (
    HEADER_1,
    HEADER_2,
    MSG_ID_ROBOT_TELEMETRY,
    MSG_ID_BMS_TELEMETRY,
    MSG_ID_SYSTEM_INFO,
    MSG_ID_ACK_NACK,
    MSG_ID_CMD_POWER_CTRL,
    MSG_ID_CMD_LED_CTRL,
    MSG_ID_CMD_SIGNAL_CTRL,
    MSG_ID_CMD_BMS_CTRL,
    MSG_ID_CMD_REQUEST_INFO,
    ACK_STATUS_OK,
    build_packet,
    BinaryPacketParser,
    calc_crc16
)

# Constants matching mobile/humanoid_mobile_mcu/config.h
SW_VER = b"MOBILE_HUMANOID_MCU_v1.0"
HW_VER = b"Mobile_Humanoid_IO_v1.0"


class VirtualMCU:
    """
    Virtual MCU Simulator providing a PTY interface identical to a real ATmega2560 MCU.
    """
    def __init__(self, start_telemetry: bool = True):
        # Create virtual PTY pair
        self.master_fd, self.slave_fd = pty.openpty()
        self.slave_name = os.ttyname(self.slave_fd)
        
        # Set raw mode on PTY
        tty.setraw(self.master_fd)
        tty.setraw(self.slave_fd)

        self.running = True
        self.telemetry_enabled = start_telemetry
        self.received_commands: List[Tuple[int, bytes]] = []
        self.lock = threading.Lock()

        # Telemetry state
        self.uptime_sec = 0
        self.btn_status = 0x02       # Start button pressed initially
        self.pc_live_status = 0x07   # All 3 PCs live (Nav, Humanoid, AI)
        self.safety_status = 0x00    # Normal safety
        self.output_status = 0x07    # Hold, 24V_1, 24V_2 ON
        self.chg_current_adc = 0.0
        self.led_current_mode = 6    # FULL_RED
        
        # BMS state
        self.bms_voltage = 48.5
        self.bms_current = -3.2
        self.bms_soc = 88.0
        self.bms_soh = 99.0
        self.bms_temp = 25.4
        self.bms_valid = 1

        self.parser = BinaryPacketParser(self._on_packet_received)

        # Background worker threads
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.telem_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        
        self.rx_thread.start()
        self.telem_thread.start()

    def _on_packet_received(self, msg_id: int, payload: bytes):
        with self.lock:
            self.received_commands.append((msg_id, payload))

        if msg_id == MSG_ID_CMD_REQUEST_INFO:
            # Respond with MSG_ID_SYSTEM_INFO (0x03)
            sw_field = SW_VER.ljust(24, b"\x00")
            hw_field = HW_VER.ljust(24, b"\x00")
            pkt = build_packet(MSG_ID_SYSTEM_INFO, sw_field + hw_field)
            self._write_raw(pkt)

        elif msg_id in (MSG_ID_CMD_POWER_CTRL, MSG_ID_CMD_LED_CTRL, MSG_ID_CMD_SIGNAL_CTRL, MSG_ID_CMD_BMS_CTRL):
            # Respond with MSG_ID_ACK_NACK (0x04) -> ACK_STATUS_OK (0x00)
            ack_payload = struct.pack("<BB", msg_id, ACK_STATUS_OK)
            pkt = build_packet(MSG_ID_ACK_NACK, ack_payload)
            self._write_raw(pkt)

            # Update internal simulated states based on command
            if msg_id == MSG_ID_CMD_LED_CTRL and len(payload) >= 1:
                self.led_current_mode = payload[0]
            elif msg_id == MSG_ID_CMD_SIGNAL_CTRL and len(payload) >= 4:
                # payload: lamp_red, lamp_grn, lamp_yel, buzzer, tm_remote
                if payload[0] == 1: self.output_status |= (1 << 4)
                elif payload[0] == 0: self.output_status &= ~(1 << 4)
                if payload[3] == 1: self.output_status |= (1 << 7)
                elif payload[3] == 0: self.output_status &= ~(1 << 7)

    def _write_raw(self, data: bytes):
        if not self.running:
            return
        try:
            os.write(self.master_fd, data)
        except OSError:
            pass

    def _rx_loop(self):
        while self.running:
            try:
                data = os.read(self.master_fd, 64)
                if not data:
                    break
                for b in data:
                    self.parser.parse_byte(b)
            except OSError:
                break

    def _telemetry_loop(self):
        step = 0
        while self.running:
            if self.telemetry_enabled:
                # 1. 20Hz Robot Telemetry
                telem_payload = struct.pack(
                    "<BBBBfBI",
                    self.btn_status,
                    self.pc_live_status,
                    self.safety_status,
                    self.output_status,
                    self.chg_current_adc,
                    self.led_current_mode,
                    self.uptime_sec
                )
                pkt = build_packet(MSG_ID_ROBOT_TELEMETRY, telem_payload)
                self._write_raw(pkt)

                # 2. 1Hz BMS Telemetry (every 20 steps of 50ms = 1000ms)
                if step % 20 == 0:
                    self.uptime_sec += 1
                    bms_payload = struct.pack(
                        "<fffffB",
                        self.bms_voltage,
                        self.bms_current,
                        self.bms_soc,
                        self.bms_soh,
                        self.bms_temp,
                        self.bms_valid
                    )
                    bms_pkt = build_packet(MSG_ID_BMS_TELEMETRY, bms_payload)
                    self._write_raw(bms_pkt)

                step += 1
            time.sleep(0.05) # 50ms period (20Hz)

    def get_received_commands(self) -> List[Tuple[int, bytes]]:
        with self.lock:
            return list(self.received_commands)

    def clear_received_commands(self):
        with self.lock:
            self.received_commands.clear()

    def set_safety_emergency(self, ems: bool):
        if ems:
            self.safety_status |= (1 << 0)
        else:
            self.safety_status &= ~(1 << 0)

    def set_bms_telemetry(self, v: float, c: float, soc: float, soh: float, temp: float, valid: int = 1):
        self.bms_voltage = v
        self.bms_current = c
        self.bms_soc = soc
        self.bms_soh = soh
        self.bms_temp = temp
        self.bms_valid = valid

    def inject_noise(self, noise_bytes: bytes):
        """Inject arbitrary noise bytes to test parser resilience"""
        self._write_raw(noise_bytes)

    def close(self):
        self.running = False
        self.telemetry_enabled = False
        try:
            os.close(self.master_fd)
        except OSError:
            pass
        try:
            os.close(self.slave_fd)
        except OSError:
            pass
