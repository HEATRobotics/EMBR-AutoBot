"""
ODESC/ODrive 3.6 configuration — firmware 0.5.6

Motor: Eagle Power LA8308 KV130, AXIS 1
Encoder: AS5047P using SPI
Control: Torque control
Host communication: CANSimple, with USB used for setup

This script applies settings and saves/reboots the board.
It does not request motor movement or calibration.

Keep external CAN command senders stopped during setup.
Existing motor-driver and encoder faults still require diagnosis.
"""

import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    ENCODER_MODE_SPI_ABS_AMS,
    CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_PASSTHROUGH,
)
from odrive.utils import dump_errors


# ============================================================
# 1. SETTINGS
# ============================================================

# Motor settings: confirm pole count and ratings for your motor.
MOTOR_POLE_PAIRS = 20
CALIBRATION_CURRENT = 5.0       # A
MOTOR_CURRENT_LIMIT = 20.0     # A; supplied development setting
MOTOR_KV = 130
TORQUE_CONSTANT = 8.27 / MOTOR_KV  # Approximate Nm/A

# SPI encoder settings.
# The encoder CS wire must physically connect to GPIO 1.
ENCODER_CS_GPIO = 1
ENCODER_CPR = 16384
ENCODER_BANDWIDTH = 1000.0     # rad/s

# Motor velocity limit.
VELOCITY_LIMIT = 5.0           # turns/s = 300 motor RPM

# CAN settings: match the host and every device on the bus.
CAN_BITRATE = 500_000          # bit/s
CAN_SIMPLE_PROTOCOL = 1       # Legacy CANSimple protocol flag

# Both axes need unique IDs, including the unused axis.
# These IDs must also be unique across the whole CAN network.
AXIS0_CAN_NODE_ID = 1
AXIS1_CAN_NODE_ID = 2

CAN_HEARTBEAT_MS = 100         # 10 heartbeat messages/s per axis
CAN_ENCODER_MS = 20            # AXIS 1 telemetry: 50 messages/s


# ============================================================
# 2. VALIDATE CAN SETTINGS
# ============================================================

if CAN_BITRATE not in (125_000, 250_000, 500_000, 1_000_000):
    raise ValueError("Unsupported CAN bitrate.")

for node_id in (AXIS0_CAN_NODE_ID, AXIS1_CAN_NODE_ID):
    if not isinstance(node_id, int) or not 0 <= node_id <= 63:
        raise ValueError("CAN node IDs must be integers from 0 to 63.")

if AXIS0_CAN_NODE_ID == AXIS1_CAN_NODE_ID:
    raise ValueError("AXIS 1 and axis 1 must have different CAN node IDs.")


# ============================================================
# 3. CONNECT
# ============================================================

print("Connecting to ODrive...")

# Reuse the board connected as dev0 inside odrivetool.
# When run as a standalone Python script, discover it over USB.
odrv0 = globals().get("dev0")

if odrv0 is None:
    odrv0 = odrive.find_any(timeout=15)

version = (
    odrv0.fw_version_major,
    odrv0.fw_version_minor,
    odrv0.fw_version_revision,
)

if version != (0, 5, 6):
    raise RuntimeError(
        f"This configuration targets firmware 0.5.6; found {version}."
    )

axis = odrv0.axis1
other_axis = odrv0.axis0

if any(
    a.current_state != AXIS_STATE_IDLE
    for a in (axis, other_axis)
):
    raise RuntimeError("Both axes must be IDLE before configuration.")

print(f"Connected. Supply voltage: {odrv0.vbus_voltage:.2f} V")
print("Existing errors:")
dump_errors(odrv0)


# ============================================================
# 4. DISABLE AUTOMATIC STARTUP MOVEMENT
# ============================================================

for a in (axis, other_axis):
    a.config.startup_closed_loop_control = False
    a.config.startup_motor_calibration = False
    a.config.startup_encoder_index_search = False
    a.config.startup_encoder_offset_calibration = False

    a.controller.input_torque = 0.0


# ============================================================
# 5. MOTOR CONFIGURATION — AXIS 1
# ============================================================

axis.motor.config.pole_pairs = MOTOR_POLE_PAIRS
axis.motor.config.calibration_current = CALIBRATION_CURRENT
axis.motor.config.current_lim = MOTOR_CURRENT_LIMIT
axis.motor.config.torque_constant = TORQUE_CONSTANT

print("\nMotor configuration:")
print(f"  Pole pairs: {MOTOR_POLE_PAIRS}")
print(f"  Calibration current: {CALIBRATION_CURRENT} A")
print(f"  Current limit: {MOTOR_CURRENT_LIMIT} A")
print(f"  Torque constant: {TORQUE_CONSTANT:.4f} Nm/A")


# ============================================================
# 6. SPI ENCODER CONFIGURATION — AXIS 1
# ============================================================

# CAN host communication does not replace encoder SPI feedback.
axis.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
axis.encoder.config.abs_spi_cs_gpio_pin = ENCODER_CS_GPIO
axis.encoder.config.cpr = ENCODER_CPR
axis.encoder.config.bandwidth = ENCODER_BANDWIDTH

print("\nEncoder configuration:")
print("  Encoder: AS5047P")
print("  Interface: SPI")
print(f"  CS GPIO: {ENCODER_CS_GPIO}")
print(f"  CPR: {ENCODER_CPR}")
print(f"  Bandwidth: {ENCODER_BANDWIDTH} rad/s")


# ============================================================
# 7. TORQUE CONTROL CONFIGURATION — AXIS 1
# ============================================================

axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

axis.controller.config.vel_limit = VELOCITY_LIMIT
axis.controller.config.enable_torque_mode_vel_limit = True

print("\nController configuration:")
print("  Control mode: Torque control")
print("  Input mode: Passthrough")
print(f"  Velocity limit: {VELOCITY_LIMIT} motor turns/s")


# ============================================================
# 8. CAN COMMUNICATION CONFIGURATION
# ============================================================

# Board-wide CAN settings.
# CANSimple is a different protocol from CANopen.
odrv0.can.config.protocol = CAN_SIMPLE_PROTOCOL
odrv0.can.config.baud_rate = CAN_BITRATE

# AXIS 1: capstan motor.
axis.config.can.node_id = AXIS0_CAN_NODE_ID
axis.config.can.is_extended = False
axis.config.can.heartbeat_rate_ms = CAN_HEARTBEAT_MS
axis.config.can.encoder_rate_ms = CAN_ENCODER_MS

# Axis 0: reserve a separate ID even when unused.
other_axis.config.can.node_id = AXIS1_CAN_NODE_ID
other_axis.config.can.is_extended = False
other_axis.config.can.heartbeat_rate_ms = CAN_HEARTBEAT_MS
other_axis.config.can.encoder_rate_ms = 0

print("\nCAN configuration:")
print("  Protocol: CANSimple")
print(f"  Bitrate: {CAN_BITRATE} bit/s")
print("  Identifier format: Standard 11-bit")
print(f"  AXIS 1 node ID: {AXIS1_CAN_NODE_ID}")
print(f"  Axis 0 node ID: {AXIS0_CAN_NODE_ID}")
print(f"  Heartbeat interval: {CAN_HEARTBEAT_MS} ms")
print(f"  AXIS 1 encoder telemetry interval: {CAN_ENCODER_MS} ms")

# Watchdog settings are left unchanged.
# Your runtime controller must manage command refresh/watchdog feeding.
# Transmitting heartbeat messages does not feed the watchdog.


# ============================================================
# 9. DIAGNOSTICS AND SAVE
# ============================================================

# Preserve errors for diagnosis rather than hiding them.
print("\nErrors before saving:")
dump_errors(odrv0)

print("\nSaving configuration...")
print("The board will reboot; USB may disconnect and reconnect.")
print("Calibration and motor tests must wait until faults are resolved.")

odrv0.save_configuration()