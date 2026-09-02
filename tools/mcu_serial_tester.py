#!/usr/bin/env python3
"""
Mobile Humanoid MCU Binary Serial Protocol Tester & Simulator Tool
"""

import sys
import time
import struct
import argparse
import threading
from typing import Optional, Callable

# Protocol Constants
HEADER_1 = 0xAA
HEADER_2 = 0x55

# Message IDs
MSG_ID_ROBOT_TELEMETRY  = 0x01
MSG_ID_BMS_TELEMETRY    = 0x02
MSG_ID_SYSTEM_INFO      = 0x03
MSG_ID_ACK_NACK         = 0x04
MSG_ID_CMD_POWER_CTRL   = 0x10
MSG_ID_CMD_LED_CTRL     = 0x11
MSG_ID_CMD_SIGNAL_CTRL  = 0x12
MSG_ID_CMD_BMS_CTRL     = 0x13
MSG_ID_CMD_REQUEST_INFO = 0x14

ACK_CODES = {
    0x00: "OK",
    0x01: "CRC_ERROR",
    0x02: "INVALID_LEN",
    0x03: "INVALID_CMD",
    0x04: "EXEC_FAILED"
}


def calc_crc16(data: bytes) -> int:
    """CRC-16-CCITT (Poly: 0x1021, Init: 0xFFFF)"""
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
    """Build a complete binary packet with Header and CRC16"""
    length = len(payload)
    header_and_body = struct.pack("<BBBB", HEADER_1, HEADER_2, msg_id, length) + payload
    crc = calc_crc16(header_and_body)
    return header_and_body + struct.pack("<H", crc)


class BinaryPacketParser:
    """FSM Packet Parser for binary protocol"""
    def __init__(self, on_packet_callback: Callable[[int, bytes], None]):
        self.callback = on_packet_callback
        self.state = 0
        self.msg_id = 0
        self.length = 0
        self.payload = bytearray()
        self.crc_bytes = bytearray()

    def parse_byte(self, byte: int):
        if self.state == 0:  # Header 1
            if byte == HEADER_1:
                self.state = 1
        elif self.state == 1:  # Header 2
            if byte == HEADER_2:
                self.state = 2
            elif byte == HEADER_1:
                self.state = 1
            else:
                self.state = 0
        elif self.state == 2:  # Msg ID
            self.msg_id = byte
            self.state = 3
        elif self.state == 3:  # Length
            self.length = byte
            self.payload = bytearray()
            if self.length == 0:
                self.crc_bytes = bytearray()
                self.state = 5
            elif self.length > 64:
                if byte == HEADER_1:
                    self.state = 1
                else:
                    self.state = 0
            else:
                self.state = 4
        elif self.state == 4:  # Payload
            self.payload.append(byte)
            if len(self.payload) >= self.length:
                self.crc_bytes = bytearray()
                self.state = 5
        elif self.state == 5:  # CRC Low
            self.crc_bytes.append(byte)
            self.state = 6
        elif self.state == 6:  # CRC High
            self.crc_bytes.append(byte)
            received_crc = self.crc_bytes[0] | (self.crc_bytes[1] << 8)
            
            # Verify CRC
            check_buf = struct.pack("<BBBB", HEADER_1, HEADER_2, self.msg_id, self.length) + bytes(self.payload)
            computed_crc = calc_crc16(check_buf)
            
            if received_crc == computed_crc:
                self.callback(self.msg_id, bytes(self.payload))
            else:
                print(f"[PARSER WARN] CRC Mismatch! Expected 0x{computed_crc:04X}, Got 0x{received_crc:04X}")
            
            self.state = 0


def decode_robot_telemetry(payload: bytes):
    if len(payload) < 13:
        print(f"[ERR] Telemetry payload too short: {len(payload)} bytes")
        return
    btn, pc, safety, out, adc, led_mode, uptime = struct.unpack("<BBBBfBI", payload)
    
    pwr_btn   = bool(btn & (1 << 0))
    start_btn = bool(btn & (1 << 1))
    stop_btn  = bool(btn & (1 << 2))
    manu_sw   = bool(btn & (1 << 3))

    nav_pc_live = bool(pc & (1 << 0))
    hum_pc_live = bool(pc & (1 << 1))
    ai_pc_live  = bool(pc & (1 << 2))

    safety_ems  = bool(safety & (1 << 0))
    manu_chg    = bool(safety & (1 << 1))
    is_charging = bool(safety & (1 << 2))

    pwr_hold  = bool(out & (1 << 0))
    volt24_1  = bool(out & (1 << 1))
    volt24_2  = bool(out & (1 << 2))
    chg_en    = bool(out & (1 << 3))
    lamp_r    = bool(out & (1 << 4))
    lamp_g    = bool(out & (1 << 5))
    lamp_y    = bool(out & (1 << 6))
    buzzer    = bool(out & (1 << 7))

    print("\n" + "=" * 65)
    print(f"📡 [ROBOT TELEMETRY] Uptime: {uptime}s | LED Mode: {led_mode}")
    print("-" * 65)
    print(f"🔘 Buttons  : PWR={int(pwr_btn)} | START={int(start_btn)} | STOP={int(stop_btn)} | MANU={int(manu_sw)}")
    print(f"💻 PC Live  : NAV_PC={int(nav_pc_live)} | HUMANOID_PC={int(hum_pc_live)} | AI_PC={int(ai_pc_live)}")
    print(f"🚨 Safety   : EMS={int(safety_ems)} | DOCK={int(manu_chg)} | CHARGING={int(is_charging)} | ADC={adc:.2f}A")
    print(f"⚡ Outputs  : HOLD={int(pwr_hold)} | 24V1={int(volt24_1)} | 24V2={int(volt24_2)} | CHG_EN={int(chg_en)}")
    print(f"🚨 Alarms   : TOWER[R={int(lamp_r)} G={int(lamp_g)} Y={int(lamp_y)}] | BUZZER={int(buzzer)}")
    print("=" * 65)


def decode_bms_telemetry(payload: bytes):
    if len(payload) < 21:
        return
    v, c, soc, soh, temp, is_valid = struct.unpack("<fffffB", payload)
    print(f"🔋 [BMS TELEMETRY] Valid: {bool(is_valid)} | Voltage: {v:.2f}V | Current: {c:.2f}A | SoC: {soc:.1f}% | SoH: {soh:.1f}% | Temp: {temp:.1f}°C")


def decode_ack(payload: bytes):
    if len(payload) < 2:
        return
    target_id, status = struct.unpack("<BB", payload)
    status_str = ACK_CODES.get(status, f"UNKNOWN(0x{status:02X})")
    print(f"📩 [ACK] Target MsgID: 0x{target_id:02X} -> Result: {status_str}")


def decode_sys_info(payload: bytes):
    if len(payload) < 48:
        return
    sw_ver = payload[:24].decode('utf-8', errors='ignore').strip('\x00')
    hw_ver = payload[24:48].decode('utf-8', errors='ignore').strip('\x00')
    print(f"ℹ️ [SYSTEM INFO] SW: '{sw_ver}' | HW: '{hw_ver}'")


def packet_dispatcher(msg_id: int, payload: bytes):
    if msg_id == MSG_ID_ROBOT_TELEMETRY:
        decode_robot_telemetry(payload)
    elif msg_id == MSG_ID_BMS_TELEMETRY:
        decode_bms_telemetry(payload)
    elif msg_id == MSG_ID_SYSTEM_INFO:
        decode_sys_info(payload)
    elif msg_id == MSG_ID_ACK_NACK:
        decode_ack(payload)
    else:
        print(f"📦 [CUSTOM PKT] MsgID: 0x{msg_id:02X}, Len: {len(payload)}, Data: {payload.hex()}")


# =============================================================================
# Self-Test / Verification Routine
# =============================================================================
def run_self_test():
    print("==================================================")
    print(" Running Protocol Encoding / Decoding Self-Test   ")
    print("==================================================")
    
    received_packets = []
    parser = BinaryPacketParser(lambda mid, p: received_packets.append((mid, p)))

    # Test 1: Robot Telemetry Encoding / Decoding
    test_telem = struct.pack("<BBBBfBI", 0x05, 0x07, 0x00, 0x03, 3.45, 25, 12345)
    pkt = build_packet(MSG_ID_ROBOT_TELEMETRY, test_telem)
    
    print(f"1. Test Robot Telemetry Frame ({len(pkt)} bytes): {pkt.hex()}")
    for b in pkt:
        parser.parse_byte(b)
    
    assert len(received_packets) == 1
    assert received_packets[0][0] == MSG_ID_ROBOT_TELEMETRY
    assert received_packets[0][1] == test_telem
    print("   -> Robot Telemetry Pack/Unpack: PASS ✅")

    # Test 2: Command LED Packet
    cmd_led = struct.pack("<BBBBB", 6, 255, 0, 0, 200) # FULL_RED
    pkt_cmd = build_packet(MSG_ID_CMD_LED_CTRL, cmd_led)
    print(f"2. Test LED Command Frame ({len(pkt_cmd)} bytes): {pkt_cmd.hex()}")
    for b in pkt_cmd:
        parser.parse_byte(b)
    
    assert len(received_packets) == 2
    assert received_packets[1][0] == MSG_ID_CMD_LED_CTRL
    print("   -> LED Command Pack/Unpack: PASS ✅")

    # Test 3: Noise immunity & Re-sync test
    garbage = b"\xFF\x00\xAA\x12\x34\xFE\xDD\xAA\x00\x55" # arbitrary noise bytes
    for b in garbage:
        parser.parse_byte(b)
    # now send valid system info query
    pkt_info = build_packet(MSG_ID_CMD_REQUEST_INFO, b"")
    for b in pkt_info:
        parser.parse_byte(b)
    
    assert len(received_packets) == 3
    assert received_packets[2][0] == MSG_ID_CMD_REQUEST_INFO
    print("   -> Noise Immunity & Re-sync: PASS ✅")
    print("==================================================")
    print(" All Protocol Self-Tests Passed Successfully! 🎉  ")
    print("==================================================")


def main():
    parser = argparse.ArgumentParser(description="Mobile Humanoid MCU Binary Protocol Tester")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=57600, help="Baud rate (default: 57600)")
    parser.add_argument("--test", action="store_true", help="Run protocol encoding/decoding self-test")
    args = parser.parse_args()

    if args.test:
        run_self_test()
        return

    try:
        import serial
    except ImportError:
        print("[ERROR] pyserial is required to connect to real hardware. Install with 'pip install pyserial'")
        sys.exit(1)

    print(f"Connecting to {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"[ERROR] Failed to open port {args.port}: {e}")
        print("Tip: Run with --test to verify protocol logic without hardware.")
        sys.exit(1)

    packet_parser = BinaryPacketParser(packet_dispatcher)
    stop_event = threading.Event()

    def rx_thread():
        while not stop_event.is_set():
            data = ser.read(64)
            if data:
                for b in data:
                    packet_parser.parse_byte(b)

    t = threading.Thread(target=rx_thread, daemon=True)
    t.start()

    # Request system info upon connect
    time.sleep(1.0)
    ser.write(build_packet(MSG_ID_CMD_REQUEST_INFO))

    print("\nListening for telemetry. Press Ctrl+C to exit.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nExiting...")
        stop_event.set()
        ser.close()


if __name__ == "__main__":
    main()
