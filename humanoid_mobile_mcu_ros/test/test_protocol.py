"""
Unit Tests for humanoid_mobile_mcu_ros.protocol
Validates binary serialization, CRC16, framing, and parser state machine.
"""

import struct
import pytest

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
    ACK_STATUS_CRC_ERROR,
    calc_crc16,
    build_packet,
    build_power_cmd,
    build_led_cmd,
    build_signal_cmd,
    build_bms_cmd,
    build_request_info_cmd,
    parse_robot_telemetry,
    parse_bms_telemetry,
    parse_ack,
    parse_system_info,
    BinaryPacketParser
)


class TestProtocolCRC:
    """Test CRC-16-CCITT implementation against reference values"""
    def test_crc16_known_vector(self):
        # Known standard test: "123456789" with CCITT (poly 0x1021, init 0xFFFF) -> 0x29B1
        test_data = b"123456789"
        crc = calc_crc16(test_data)
        assert crc == 0x29B1, f"Expected 0x29B1, got 0x{crc:04X}"

    def test_crc16_empty(self):
        assert calc_crc16(b"") == 0xFFFF


class TestCommandBuilders:
    """Test packet builders matching C++ structs in mobile/humanoid_mobile_mcu/protocol.h"""
    def test_build_power_cmd(self):
        # Target: Humanoid PC (1), Action: PULSE (2)
        pkt = build_power_cmd(1, 2)
        assert len(pkt) == 8 # 2(Header) + 1(ID) + 1(Len) + 2(Payload) + 2(CRC)
        assert pkt[0] == HEADER_1
        assert pkt[1] == HEADER_2
        assert pkt[2] == MSG_ID_CMD_POWER_CTRL
        assert pkt[3] == 2 # Payload len
        assert pkt[4] == 1 # Target
        assert pkt[5] == 2 # Action

        # Verify CRC
        computed_crc = calc_crc16(pkt[:6])
        recv_crc = pkt[6] | (pkt[7] << 8)
        assert computed_crc == recv_crc

    def test_build_led_cmd(self):
        # Mode: FULL_RED (6), RGB=(255, 0, 0), Brightness=200
        pkt = build_led_cmd(6, 255, 0, 0, 200)
        assert len(pkt) == 11 # 4 + 5 + 2
        assert pkt[2] == MSG_ID_CMD_LED_CTRL
        assert pkt[3] == 5
        assert pkt[4:9] == bytes([6, 255, 0, 0, 200])

    def test_build_signal_cmd(self):
        # Red=1, Grn=0, Yel=255, Buzzer=1, TM=255
        pkt = build_signal_cmd(1, 0, 255, 1, 255)
        assert len(pkt) == 11
        assert pkt[2] == MSG_ID_CMD_SIGNAL_CTRL
        assert pkt[3] == 5
        assert pkt[4:9] == bytes([1, 0, 255, 1, 255])

    def test_build_bms_cmd(self):
        pkt = build_bms_cmd(1, 0)
        assert len(pkt) == 8
        assert pkt[2] == MSG_ID_CMD_BMS_CTRL
        assert pkt[3] == 2
        assert pkt[4:6] == bytes([1, 0])

    def test_build_request_info_cmd(self):
        pkt = build_request_info_cmd()
        assert len(pkt) == 6 # 4 + 0 + 2
        assert pkt[2] == MSG_ID_CMD_REQUEST_INFO
        assert pkt[3] == 0


class TestPayloadDecoders:
    """Test payload decoders matching C++ structs in mobile/humanoid_mobile_mcu/protocol.h"""
    def test_parse_robot_telemetry(self):
        # Struct: btn(1), pc(1), safety(1), out(1), chg_adc(float 4), led_mode(1), uptime(uint32 4) -> 13B
        btn = 0x05       # Bit 0(PWR) + Bit 2(STOP)
        pc = 0x07        # All 3 PCs live
        safety = 0x01    # EMS active
        out = 0x83       # HOLD(bit0), 24V1(bit1), BUZZER(bit7)
        adc = 3.45
        led_mode = 25    # BLINK_RED
        uptime = 123456

        payload = struct.pack("<BBBBfBI", btn, pc, safety, out, adc, led_mode, uptime)
        data = parse_robot_telemetry(payload)
        assert data is not None

        assert data["pwr_btn"] is True
        assert data["start_btn"] is False
        assert data["stop_btn"] is True
        assert data["manu_sw"] is False

        assert data["nav_pc_live"] is True
        assert data["humanoid_pc_live"] is True
        assert data["ai_pc_live"] is True

        assert data["safety_ems"] is True
        assert data["manu_chg_dock"] is False
        assert data["is_charging"] is False

        assert data["pwr_hold"] is True
        assert data["volt24v_1"] is True
        assert data["volt24v_2"] is False
        assert data["buzzer"] is True

        assert pytest.approx(data["chg_current_adc"], 0.01) == 3.45
        assert data["led_current_mode"] == 25
        assert data["uptime_sec"] == 123456

    def test_parse_bms_telemetry(self):
        # Struct: v(float), c(float), soc(float), soh(float), temp(float), valid(uint8) -> 21B
        payload = struct.pack("<fffffB", 51.2, -12.5, 75.5, 95.0, 31.2, 1)
        data = parse_bms_telemetry(payload)
        assert data is not None

        assert pytest.approx(data["voltage"], 0.01) == 51.2
        assert pytest.approx(data["current"], 0.01) == -12.5
        assert pytest.approx(data["soc"], 0.01) == 75.5
        assert pytest.approx(data["soh"], 0.01) == 95.0
        assert pytest.approx(data["temp"], 0.01) == 31.2
        assert data["is_valid"] is True

    def test_parse_ack(self):
        payload = struct.pack("<BB", MSG_ID_CMD_LED_CTRL, ACK_STATUS_OK)
        data = parse_ack(payload)
        assert data is not None
        assert data["target_msg_id"] == MSG_ID_CMD_LED_CTRL
        assert data["status_code"] == ACK_STATUS_OK
        assert data["status_str"] == "OK"

    def test_parse_system_info(self):
        sw = b"MOBILE_HUMANOID_MCU_v1.0".ljust(24, b"\x00")
        hw = b"Mobile_Humanoid_IO_v1.0".ljust(24, b"\x00")
        data = parse_system_info(sw + hw)
        assert data is not None
        assert data["sw_ver"] == "MOBILE_HUMANOID_MCU_v1.0"
        assert data["hw_ver"] == "Mobile_Humanoid_IO_v1.0"


class TestBinaryPacketParserFSM:
    """Test Byte-stream FSM Parser robustness and error recovery"""
    def test_clean_packet_stream(self):
        received = []
        parser = BinaryPacketParser(lambda mid, p: received.append((mid, p)))

        pkt1 = build_power_cmd(1, 2)
        pkt2 = build_led_cmd(6, 255, 0, 0, 200)

        for b in pkt1 + pkt2:
            parser.parse_byte(b)

        assert len(received) == 2
        assert received[0][0] == MSG_ID_CMD_POWER_CTRL
        assert received[1][0] == MSG_ID_CMD_LED_CTRL

    def test_noise_and_resync(self):
        received = []
        parser = BinaryPacketParser(lambda mid, p: received.append((mid, p)))

        # Inject noise and false headers
        garbage = b"\x00\xFF\xAA\x12\xAA\xAA\x54\x55\xDE\xAD\xBE\xEF"
        for b in garbage:
            parser.parse_byte(b)

        # Now send valid packet
        pkt = build_request_info_cmd()
        for b in pkt:
            parser.parse_byte(b)

        assert len(received) == 1
        assert received[0][0] == MSG_ID_CMD_REQUEST_INFO

    def test_corrupted_crc_rejected(self):
        received = []
        parser = BinaryPacketParser(lambda mid, p: received.append((mid, p)))

        pkt = bytearray(build_power_cmd(1, 2))
        pkt[-1] ^= 0xFF # Corrupt CRC byte

        for b in pkt:
            parser.parse_byte(b)

        assert len(received) == 0 # Corrupted packet must be dropped
