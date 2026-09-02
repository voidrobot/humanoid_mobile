#!/usr/bin/env python3
"""
Automated Test Runner for humanoid_mobile_mcu_ros
Runs both unit tests (protocol) and E2E integration tests (bridge node + virtual MCU)
"""

import sys
import os
import time
import pytest

# ANSI Color codes for clean reporting
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
BOLD = "\033[1m"
RESET = "\033[0m"


def print_banner():
    print(f"\n{CYAN}{BOLD}{'=' * 75}{RESET}")
    print(f"{CYAN}{BOLD} 🤖 Mobile Humanoid MCU ROS 2 Bridge Node Automated Test Suite {RESET}")
    print(f"{CYAN}{BOLD}{'=' * 75}{RESET}\n")


def main():
    print_banner()

    test_dir = os.path.dirname(os.path.abspath(__file__))
    protocol_test_file = os.path.join(test_dir, "test_protocol.py")
    bridge_test_file = os.path.join(test_dir, "test_mcu_bridge_node.py")

    print(f"{BOLD}[1/2] Running Protocol Unit Tests ({protocol_test_file})...{RESET}")
    start_time = time.time()
    ret_protocol = pytest.main(["-p", "no:dash", protocol_test_file, "-v", "--tb=short"])
    protocol_duration = time.time() - start_time

    print(f"\n{BOLD}[2/2] Running ROS 2 Bridge Node E2E Integration Tests ({bridge_test_file})...{RESET}")
    start_time = time.time()
    ret_bridge = pytest.main(["-p", "no:dash", bridge_test_file, "-v", "--tb=short"])
    bridge_duration = time.time() - start_time

    print(f"\n{CYAN}{BOLD}{'=' * 75}{RESET}")
    print(f"{BOLD}📊 TEST EXECUTION SUMMARY REPORT{RESET}")
    print(f"{CYAN}{BOLD}{'=' * 75}{RESET}")

    status_proto = f"{GREEN}PASS ✅{RESET}" if ret_protocol == 0 else f"{RED}FAIL ❌{RESET}"
    status_bridge = f"{GREEN}PASS ✅{RESET}" if ret_bridge == 0 else f"{RED}FAIL ❌{RESET}"

    print(f" • Protocol Unit Tests (CRC16, Framing, FSM, Decoders) : {status_proto} ({protocol_duration:.2f}s)")
    print(f" • Bridge Node E2E Tests (ROS 2 Topics, Virtual MCU)   : {status_bridge} ({bridge_duration:.2f}s)")
    print(f"{CYAN}{BOLD}{'-' * 75}{RESET}")

    if ret_protocol == 0 and ret_bridge == 0:
        print(f"{GREEN}{BOLD}🎉 ALL TESTS PASSED SUCCESSFULLY! ZERO ERRORS DETECTED! 🎉{RESET}")
        print(f"{CYAN}{BOLD}{'=' * 75}{RESET}\n")
        sys.exit(0)
    else:
        print(f"{RED}{BOLD}⚠️ SOME TESTS FAILED. PLEASE CHECK THE OUTPUT ABOVE. ⚠️{RESET}")
        print(f"{CYAN}{BOLD}{'=' * 75}{RESET}\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
