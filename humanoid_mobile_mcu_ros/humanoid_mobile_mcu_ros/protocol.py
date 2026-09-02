"""
Binary Serial Protocol Definition & Helpers for Mobile Humanoid MCU
"""

import struct
from typing import Callable, Optional, Dict, Any, Tuple

# Header Magic Numbers
HEADER_1 = 0xAA
HEADER_2 = 0x55

# Message IDs (MCU -> PC)
MSG_ID_ROBOT_TELEMETRY  = 0x01
MSG_ID_BMS_TELEMETRY    = 0x02
MSG_ID_SYSTEM_INFO      = 0x03
MSG_ID_ACK_NACK         = 0x04

# Message IDs (PC -> MCU)
MSG_ID_CMD_POWER_CTRL   = 0x10
MSG_ID_CMD_LED_CTRL     = 0x11
MSG_ID_CMD_SIGNAL_CTRL  = 0x12
MSG_ID_CMD_BMS_CTRL     = 0x13
MSG_ID_CMD_REQUEST_INFO = 0x14

# ACK/NACK Status Codes
ACK_STATUS_OK          = 0x00
ACK_STATUS_CRC_ERROR   = 0x01
ACK_STATUS_INVALID_LEN = 0x02
ACK_STATUS_INVALID_CMD = 0x03
ACK_STATUS_EXEC_FAILED = 0x04

ACK_STATUS_STR = {
    ACK_STATUS_OK: "OK",
    ACK_STATUS_CRC_ERROR: "CRC_ERROR",
    ACK_STATUS_INVALID_LEN: "INVALID_LEN",
    ACK_STATUS_INVALID_CMD: "INVALID_CMD",
    ACK_STATUS_EXEC_FAILED: "EXEC_FAILED"
}


def calc_crc16(data: bytes) -> int:
    """Calculate CRC-16-CCITT (Polynomial: 0x1021, Initial: 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_packet(msg_id: int, payload: bytes = b"") -> bytes:
    """Wrap payload into a complete binary packet with header and CRC16"""
    length = len(payload)
    if length > 64:
        raise ValueError(f"Payload size {length} exceeds maximum 64 bytes")
    header_and_body = struct.pack("<BBBB", HEADER_1, HEADER_2, msg_id, length) + payload
    crc = calc_crc16(header_and_body)
    return header_and_body + struct.pack("<H", crc)


class BinaryPacketParser:
    """
    Byte-stream FSM Packet Parser for Mobile Humanoid MCU Protocol
    """
    STATE_HEADER_1 = 0
    STATE_HEADER_2 = 1
    STATE_MSG_ID   = 2
    STATE_LENGTH   = 3
    STATE_PAYLOAD  = 4
    STATE_CRC_L    = 5
    STATE_CRC_H    = 6

    def __init__(self, on_packet_callback: Optional[Callable[[int, bytes], None]] = None):
        self._callback = on_packet_callback
        self._state = self.STATE_HEADER_1
        self._msg_id = 0
        self._length = 0
        self._payload = bytearray()
        self._crc_bytes = bytearray()

    def set_callback(self, callback: Callable[[int, bytes], None]):
        self._callback = callback

    def parse_byte(self, byte: int):
        if self._state == self.STATE_HEADER_1:
            if byte == HEADER_1:
                self._state = self.STATE_HEADER_2

        elif self._state == self.STATE_HEADER_2:
            if byte == HEADER_2:
                self._state = self.STATE_MSG_ID
            elif byte == HEADER_1:
                self._state = self.STATE_HEADER_2  # consecutive 0xAA
            else:
                self._state = self.STATE_HEADER_1

        elif self._state == self.STATE_MSG_ID:
            self._msg_id = byte
            self._state = self.STATE_LENGTH

        elif self._state == self.STATE_LENGTH:
            self._length = byte
            self._payload = bytearray()
            if self._length == 0:
                self._crc_bytes = bytearray()
                self._state = self.STATE_CRC_L
            elif self._length > 64:
                # Invalid length, reset parser
                if byte == HEADER_1:
                    self._state = self.STATE_HEADER_2
                else:
                    self._state = self.STATE_HEADER_1
            else:
                self._state = self.STATE_PAYLOAD

        elif self._state == self.STATE_PAYLOAD:
            self._payload.append(byte)
            if len(self._payload) >= self._length:
                self._crc_bytes = bytearray()
                self._state = self.STATE_CRC_L

        elif self._state == self.STATE_CRC_L:
            self._crc_bytes.append(byte)
            self._state = self.STATE_CRC_H

        elif self._state == self.STATE_CRC_H:
            self._crc_bytes.append(byte)
            received_crc = self._crc_bytes[0] | (self._crc_bytes[1] << 8)

            check_buf = struct.pack("<BBBB", HEADER_1, HEADER_2, self._msg_id, self._length) + bytes(self._payload)
            computed_crc = calc_crc16(check_buf)

            if received_crc == computed_crc:
                if self._callback is not None:
                    self._callback(self._msg_id, bytes(self._payload))

            self._state = self.STATE_HEADER_1

        else:
            self._state = self.STATE_HEADER_1


# =============================================================================
# Command Packet Builders
# =============================================================================

def build_power_cmd(target_device: int, action: int) -> bytes:
    payload = struct.pack("<BB", target_device, action)
    return build_packet(MSG_ID_CMD_POWER_CTRL, payload)


def build_led_cmd(mode: int, r: int, g: int, b: int, brightness: int = 200) -> bytes:
    payload = struct.pack("<BBBBB", mode, r, g, b, brightness)
    return build_packet(MSG_ID_CMD_LED_CTRL, payload)


def build_signal_cmd(lamp_red: int, lamp_grn: int, lamp_yel: int, buzzer: int, tm_remote: int) -> bytes:
    payload = struct.pack("<BBBBB", lamp_red, lamp_grn, lamp_yel, buzzer, tm_remote)
    return build_packet(MSG_ID_CMD_SIGNAL_CTRL, payload)


def build_bms_cmd(bms_reset_pulse: int, chg_enable: int) -> bytes:
    payload = struct.pack("<BB", bms_reset_pulse, chg_enable)
    return build_packet(MSG_ID_CMD_BMS_CTRL, payload)


def build_request_info_cmd() -> bytes:
    return build_packet(MSG_ID_CMD_REQUEST_INFO, b"")


# =============================================================================
# Payload Decoders
# =============================================================================

def parse_robot_telemetry(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < 13:
        return None
    btn, pc, safety, out, adc, led_mode, uptime = struct.unpack("<BBBBfBI", payload[:13])

    return {
        "btn_status": btn,
        "pwr_btn": bool(btn & (1 << 0)),
        "start_btn": bool(btn & (1 << 1)),
        "stop_btn": bool(btn & (1 << 2)),
        "manu_sw": bool(btn & (1 << 3)),

        "pc_live_status": pc,
        "nav_pc_live": bool(pc & (1 << 0)),
        "humanoid_pc_live": bool(pc & (1 << 1)),
        "ai_pc_live": bool(pc & (1 << 2)),

        "safety_status": safety,
        "safety_ems": bool(safety & (1 << 0)),
        "manu_chg_dock": bool(safety & (1 << 1)),
        "is_charging": bool(safety & (1 << 2)),

        "output_status": out,
        "pwr_hold": bool(out & (1 << 0)),
        "volt24v_1": bool(out & (1 << 1)),
        "volt24v_2": bool(out & (1 << 2)),
        "chg_on_enable": bool(out & (1 << 3)),
        "lamp_red": bool(out & (1 << 4)),
        "lamp_grn": bool(out & (1 << 5)),
        "lamp_yel": bool(out & (1 << 6)),
        "buzzer": bool(out & (1 << 7)),

        "chg_current_adc": float(adc),
        "led_current_mode": int(led_mode),
        "uptime_sec": int(uptime)
    }


def parse_bms_telemetry(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < 21:
        return None
    v, c, soc, soh, temp, is_valid = struct.unpack("<fffffB", payload[:21])
    return {
        "voltage": float(v),
        "current": float(c),
        "soc": float(soc),
        "soh": float(soh),
        "temp": float(temp),
        "is_valid": bool(is_valid)
    }


def parse_ack(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < 2:
        return None
    target_id, status = struct.unpack("<BB", payload[:2])
    return {
        "target_msg_id": target_id,
        "status_code": status,
        "status_str": ACK_STATUS_STR.get(status, f"UNKNOWN(0x{status:02X})")
    }


def parse_system_info(payload: bytes) -> Optional[Dict[str, Any]]:
    if len(payload) < 48:
        return None
    sw_ver = payload[:24].decode("utf-8", errors="ignore").strip("\x00")
    hw_ver = payload[24:48].decode("utf-8", errors="ignore").strip("\x00")
    return {
        "sw_ver": sw_ver,
        "hw_ver": hw_ver
    }
