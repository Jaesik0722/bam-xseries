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

# --- RAM ------------------------------------------------------------------------------
ADDR_TORQUE_ENABLE = 64       # 1 byte
ADDR_POSITION_D_GAIN = 80     # 2 bytes
ADDR_POSITION_I_GAIN = 82     # 2 bytes
ADDR_POSITION_P_GAIN = 84     # 2 bytes
ADDR_GOAL_POSITION = 116      # 4 bytes, 1 pulse
ADDR_PRESENT_CURRENT = 126    # 2 bytes, 2.69 mA
ADDR_PRESENT_VELOCITY = 128   # 4 bytes, 0.229 rev/min
ADDR_PRESENT_POSITION = 132   # 4 bytes, 1 pulse
ADDR_PRESENT_INPUT_VOLTAGE = 144  # 2 bytes, 0.1 V
ADDR_PRESENT_TEMPERATURE = 146    # 1 byte, 1 degC

# A single contiguous read from Present Current through Present Temperature keeps the
# sample atomic: position, velocity and temperature then belong to the same control cycle.
ADDR_READ_BLOCK = ADDR_PRESENT_CURRENT
LEN_READ_BLOCK = ADDR_PRESENT_TEMPERATURE - ADDR_PRESENT_CURRENT + 1  # 21 bytes

COUNTS_PER_REV = 4096
CURRENT_UNIT_A = 2.69e-3            # A per count
VELOCITY_UNIT_RAD_S = 0.229 * 2 * np.pi / 60.0
VOLTAGE_UNIT_V = 0.1


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

    # -- logging ------------------------------------------------------------------------
    def read_data(self) -> dict:
        """One sample: position, speed, load, input voltage and temperature.

        ``load`` is the measured current in amperes rather than the Protocol 1.0 "load"
        percentage, which the X series does not provide. The identification does not use
        this channel, but it is recorded because it makes an independent check of the
        fitted torque constant possible afterwards.
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
        current = _signed(word(ADDR_PRESENT_CURRENT - base, 2), 16) * CURRENT_UNIT_A
        velocity = _signed(word(ADDR_PRESENT_VELOCITY - base, 4), 32) * VELOCITY_UNIT_RAD_S
        position = word(ADDR_PRESENT_POSITION - base, 4)
        position = 2 * np.pi * ((position / COUNTS_PER_REV) - 0.5)
        volts = word(ADDR_PRESENT_INPUT_VOLTAGE - base, 2) * VOLTAGE_UNIT_V
        temp = word(ADDR_PRESENT_TEMPERATURE - base, 1)

        return {
            "position": position,
            "speed": velocity,
            "load": current,
            "input_volts": volts,
            "temp": temp,
        }
