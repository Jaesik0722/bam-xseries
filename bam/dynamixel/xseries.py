# Copyright 2026 Jaesik Jeong
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Added to Better Actuator Models (Copyright 2025 Marc Duclusaud & Gregoire Passault)
# to support the DYNAMIXEL X series.

# Protocol 2.0 driver for the DYNAMIXEL X series (XH540, XM540, XM430).
#
# BAM ships a Protocol 1.0 driver (dynamixel.py) whose control-table addresses belong to the
# MX series. The X series uses Protocol 2.0 and a different table, so a separate driver is
# needed. The interface is the one the recorder expects: set_p_gain, set_torque,
# set_goal_position, read_data.
#
# Addresses and units are from the ROBOTIS e-Manual control table, which the XH540-W270,
# XM540-W270 and XM430-W350 share.

import os

import numpy as np
from dynamixel_sdk import PacketHandler, PortHandler

# --- EEPROM ---------------------------------------------------------------------------
ADDR_TEMPERATURE_LIMIT = 31   # 1 byte, 1 degC
ADDR_PWM_LIMIT = 36           # 2 bytes, 0.113 %
ADDR_CURRENT_LIMIT = 38       # 2 bytes, 2.69 mA

# Every register that changes how the firmware turns a goal position into a PWM duty, plus
# the ones that identify the unit and the bus. The recorder only writes the P gain and
# Torque Enable, so everything else is whatever is stored in the servo; it is snapshotted
# into each log so that the identification can be tied to the configuration it ran under.
# (name, address, size in bytes, signed)
SNAPSHOT_REGISTERS = [
    ("model_number", 0, 2, False),
    ("firmware_version", 6, 1, False),
    ("id", 7, 1, False),
    ("baud_rate", 8, 1, False),
    ("return_delay_time", 9, 1, False),      # 2 us per count
    ("drive_mode", 10, 1, False),            # bit 2: time-based profile
    ("operating_mode", 11, 1, False),        # 3 = position control
    ("homing_offset", 20, 4, True),
    ("temperature_limit", 31, 1, False),
    ("max_voltage_limit", 32, 2, False),     # 0.1 V
    ("min_voltage_limit", 34, 2, False),     # 0.1 V
    ("pwm_limit", 36, 2, False),             # 885 = 100 %
    ("current_limit", 38, 2, False),         # 2.69 mA
    ("velocity_limit", 44, 4, False),        # 0.229 rev/min
    ("max_position_limit", 48, 4, False),
    ("min_position_limit", 52, 4, False),
    ("shutdown", 63, 1, False),
    ("status_return_level", 68, 1, False),
    ("velocity_i_gain", 76, 2, False),
    ("velocity_p_gain", 78, 2, False),
    ("position_d_gain", 80, 2, False),
    ("position_i_gain", 82, 2, False),
    ("position_p_gain", 84, 2, False),
    ("feedforward_2nd_gain", 88, 2, False),
    ("feedforward_1st_gain", 90, 2, False),
    ("bus_watchdog", 98, 1, False),
    ("goal_pwm", 100, 2, True),
    ("goal_current", 102, 2, True),
    ("profile_acceleration", 108, 4, False),
    ("profile_velocity", 112, 4, False),
]

# Values the identification assumes. A mismatch does not stop anything, but it means the
# fitted model describes a different controller from the one in xseries_actuator.py.
#   pwm_limit            -> X_PWM_LIMIT in xseries_actuator.py, enters error_gain
#   operating_mode       -> the model is a voltage-controlled position loop
#   position_i/d, ff     -> the model has a pure P loop
#   profile_*            -> a non-zero profile makes the firmware smooth the goal, which the
#                           model does not simulate
EXPECTED_REGISTERS = {
    "operating_mode": 3,
    "pwm_limit": 885,
    "position_i_gain": 0,
    "position_d_gain": 0,
    "feedforward_1st_gain": 0,
    "feedforward_2nd_gain": 0,
    "profile_acceleration": 0,
    "profile_velocity": 0,
}


def check_registers(registers: dict) -> list[str]:
    """Return one message per register that differs from what the model assumes.

    ``position_p_gain`` is not checked: the recorder sets it per log.
    """
    problems = []
    for name, expected in EXPECTED_REGISTERS.items():
        if name in registers and registers[name] != expected:
            problems.append(f"{name} = {registers[name]} (model assumes {expected})")
    drive_mode = registers.get("drive_mode")
    if drive_mode is not None and drive_mode & 0b100:
        problems.append(f"drive_mode = {drive_mode}: time-based profile is enabled")
    goal_pwm, pwm_limit = registers.get("goal_pwm"), registers.get("pwm_limit")
    if goal_pwm is not None and pwm_limit is not None and abs(goal_pwm) < pwm_limit:
        problems.append(
            f"goal_pwm = {goal_pwm} is below pwm_limit = {pwm_limit} and may cap the duty")
    return problems

# --- RAM ------------------------------------------------------------------------------
ADDR_TORQUE_ENABLE = 64       # 1 byte
ADDR_POSITION_D_GAIN = 80     # 2 bytes
ADDR_POSITION_I_GAIN = 82     # 2 bytes
ADDR_POSITION_P_GAIN = 84     # 2 bytes
ADDR_GOAL_POSITION = 116      # 4 bytes, 1 pulse
ADDR_PRESENT_PWM = 124        # 2 bytes, 0.113 % (885 = 100 %)
ADDR_PRESENT_CURRENT = 126    # 2 bytes, 2.69 mA
ADDR_PRESENT_VELOCITY = 128   # 4 bytes, 0.229 rev/min
ADDR_PRESENT_POSITION = 132   # 4 bytes, 1 pulse
ADDR_PRESENT_INPUT_VOLTAGE = 144  # 2 bytes, 0.1 V
ADDR_PRESENT_TEMPERATURE = 146    # 1 byte, 1 degC

# A single contiguous read from Present PWM through Present Temperature keeps the sample
# atomic: PWM, position, velocity and temperature then belong to the same control cycle.
ADDR_READ_BLOCK = ADDR_PRESENT_PWM
LEN_READ_BLOCK = ADDR_PRESENT_TEMPERATURE - ADDR_PRESENT_PWM + 1  # 23 bytes

COUNTS_PER_REV = 4096
CURRENT_UNIT_A = 2.69e-3            # A per count
VELOCITY_UNIT_RAD_S = 0.229 * 2 * np.pi / 60.0
VOLTAGE_UNIT_V = 0.1
PWM_FULL_SCALE = 885                # Present PWM count at 100 % duty


def _signed(value: int, bits: int) -> int:
    """Interpret an unsigned control-table word as two's complement."""
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


class DynamixelXSeries:
    """Protocol 2.0 actuator on a single bus, addressed by id.

    The position convention matches the Protocol 1.0 driver: zero radians is the middle of
    the travel, so a raw count of 2048 reads as 0.
    """

    def __init__(self, port: str, id: int = 1, baudrate: int = 1_000_000,
                 low_latency: bool = True):
        self.id = id

        if low_latency and port.startswith("/dev/tty"):
            # Without this the USB latency timer dominates the sample period and the
            # recorded velocity is quantised by the transport rather than by the encoder.
            if os.system(f"setserial {port} low_latency") != 0:
                raise RuntimeError(
                    f"could not set low latency on {port} "
                    "(try: sudo apt install setserial, and check you own the port)")

        self.portHandler = PortHandler(port)
        self.packetHandler = PacketHandler(2.0)
        if not self.portHandler.openPort():
            raise RuntimeError(f"could not open {port}")
        if not self.portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"could not set baudrate {baudrate} on {port}")

    # -- configuration ------------------------------------------------------------------
    def _write(self, addr: int, value: int, size: int):
        fn = {1: self.packetHandler.write1ByteTxRx,
              2: self.packetHandler.write2ByteTxRx,
              4: self.packetHandler.write4ByteTxRx}[size]
        result, error = fn(self.portHandler, self.id, addr, value)
        if result != 0 or error != 0:
            raise RuntimeError(
                f"write to {addr} failed: "
                f"{self.packetHandler.getTxRxResult(result)} "
                f"{self.packetHandler.getRxPacketError(error)}")

    def _read(self, addr: int, size: int) -> int:
        fn = {1: self.packetHandler.read1ByteTxRx,
              2: self.packetHandler.read2ByteTxRx,
              4: self.packetHandler.read4ByteTxRx}[size]
        value, result, error = fn(self.portHandler, self.id, addr)
        if result != 0 or error != 0:
            raise RuntimeError(f"read from {addr} failed")
        return value

    def set_p_gain(self, gain: int):
        """Position P Gain, 0-16383. The firmware default for these units is 800."""
        self._write(ADDR_POSITION_P_GAIN, int(gain), 2)

    def set_pid_gains(self, p: int, i: int, d: int):
        self._write(ADDR_POSITION_D_GAIN, int(d), 2)
        self._write(ADDR_POSITION_I_GAIN, int(i), 2)
        self._write(ADDR_POSITION_P_GAIN, int(p), 2)

    def set_torque(self, enable: bool):
        self._write(ADDR_TORQUE_ENABLE, 1 if enable else 0, 1)

    def set_goal_position(self, position: float):
        """Position in radians, zero at the centre of travel."""
        raw = int(round(COUNTS_PER_REV * (position / (2 * np.pi) + 0.5)))
        self._write(ADDR_GOAL_POSITION, int(np.clip(raw, 0, COUNTS_PER_REV - 1)), 4)

    def read_temperature_limit(self) -> int:
        return self._read(ADDR_TEMPERATURE_LIMIT, 1)

    def read_registers(self) -> dict:
        """Read every register in SNAPSHOT_REGISTERS. Reads only; nothing is written."""
        registers = {}
        for name, addr, size, signed in SNAPSHOT_REGISTERS:
            value = self._read(addr, size)
            registers[name] = _signed(value, 8 * size) if signed else value
        return registers

    # -- logging ------------------------------------------------------------------------
    def read_data(self) -> dict:
        """One sample: position, speed, load, PWM, input voltage and temperature.

        ``load`` is the measured current in amperes rather than the Protocol 1.0 "load"
        percentage, which the X series does not provide. ``pwm`` is the duty cycle the
        firmware applied, as a signed fraction of full scale. The identification uses
        neither channel. ``load`` makes an independent check of the fitted torque constant
        possible afterwards, and ``pwm`` against the position error checks ``error_gain``
        directly (PROTOCOL.md section 9).
        """
        data, result, error = self.packetHandler.readTxRx(
            self.portHandler, self.id, ADDR_READ_BLOCK, LEN_READ_BLOCK)
        if result != 0 or error != 0:
            raise RuntimeError(
                f"read failed: {self.packetHandler.getTxRxResult(result)} "
                f"{self.packetHandler.getRxPacketError(error)}")

        def word(offset, size):
            return int.from_bytes(bytes(data[offset:offset + size]), "little")

        base = ADDR_READ_BLOCK
        pwm = _signed(word(ADDR_PRESENT_PWM - base, 2), 16) / PWM_FULL_SCALE
        current = _signed(word(ADDR_PRESENT_CURRENT - base, 2), 16) * CURRENT_UNIT_A
        velocity = _signed(word(ADDR_PRESENT_VELOCITY - base, 4), 32) * VELOCITY_UNIT_RAD_S
        # Signed: below count 0 (possible after a homing offset or in extended modes) the
        # register wraps to 2**32 - n.
        position = _signed(word(ADDR_PRESENT_POSITION - base, 4), 32)
        position = 2 * np.pi * ((position / COUNTS_PER_REV) - 0.5)
        volts = word(ADDR_PRESENT_INPUT_VOLTAGE - base, 2) * VOLTAGE_UNIT_V
        temp = word(ADDR_PRESENT_TEMPERATURE - base, 1)

        return {
            "position": position,
            "speed": velocity,
            "load": current,
            "pwm": pwm,
            "input_volts": volts,
            "temp": temp,
        }
