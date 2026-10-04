"""
ODrive 3.6 Configuration
========================

Motor:
    Eagle Power LA8308 KV130

Encoder:
    AS5047P magnetic encoder

Encoder interface:
    SPI

Control:
    Torque control

Purpose:
    Configure the ODrive before motor/encoder calibration
    and torque-control testing.

This file does NOT command the motor to rotate.
It only applies the hardware/control configuration.
"""

import odrive
from odrive.enums import *


# ============================================================
# 1. CONNECT TO ODRIVE
# ============================================================

print("Searching for ODrive 3.6...")

odrv0 = odrive.find_any()

print("ODrive connected.")


# ============================================================
# 2. SELECT MOTOR AXIS
# ============================================================

# ODrive 3.6 has two motor axes.
# We are using axis 0 for the Capstan Eagle Power motor.

axis = odrv0.axis0


# ============================================================
# 3. MOTOR CONFIGURATION
# ============================================================

# Motor:
# Eagle Power LA8308 KV130
#
# The 8308 motor family is reported as 36N40P:
#
#   36 stator slots
#   40 rotor poles
#
# Pole pairs = 40 / 2 = 20

MOTOR_POLE_PAIRS = 20

axis.motor.config.pole_pairs = MOTOR_POLE_PAIRS


# ------------------------------------------------------------
# Calibration current
# ------------------------------------------------------------
#
# This is the current ODrive uses during motor calibration.
#
# Start conservatively.
#
# This is NOT the maximum current the motor can draw.

CALIBRATION_CURRENT = 5.0

axis.motor.config.calibration_current = CALIBRATION_CURRENT


# ------------------------------------------------------------
# Motor current limit
# ------------------------------------------------------------
#
# This limits the motor current during normal operation.
#
# The Eagle Power 8308 family has been reported with current
# ratings up to approximately 38 A depending on variant.
#
# We are intentionally starting below that value.
#
# IMPORTANT:
# 20 A is a conservative DEVELOPMENT starting value,
# not a confirmed continuous rating for the LA8308 KV130.

MOTOR_CURRENT_LIMIT = 20.0

axis.motor.config.current_lim = MOTOR_CURRENT_LIMIT


# ------------------------------------------------------------
# Motor torque constant
# ------------------------------------------------------------
#
# Approximate relationship:
#
#     torque_constant ≈ 8.27 / KV
#
# For KV130:
#
#     8.27 / 130 ≈ 0.0636 Nm/A

MOTOR_KV = 130

TORQUE_CONSTANT = 8.27 / MOTOR_KV

axis.motor.config.torque_constant = TORQUE_CONSTANT


print()
print("Motor configuration:")
print(f"  Motor: Eagle Power LA8308 KV130")
print(f"  Pole pairs: {MOTOR_POLE_PAIRS}")
print(f"  Calibration current: {CALIBRATION_CURRENT} A")
print(f"  Current limit: {MOTOR_CURRENT_LIMIT} A")
print(f"  Torque constant: {TORQUE_CONSTANT:.4f} Nm/A")


# ============================================================
# 4. ENCODER CONFIGURATION
# ============================================================

# Encoder:
# AMS/Infineon AS5047P
#
# The AS5047P is a 14-bit absolute magnetic encoder.
#
# We are using its SPI interface rather than A/B/Z.
#
# SPI encoder:
#
#     SCK
#     MISO
#     MOSI
#     CS
#
# The encoder position is used as feedback for the ODrive
# motor controller.


# ------------------------------------------------------------
# Encoder mode
# ------------------------------------------------------------

axis.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS


# ------------------------------------------------------------
# SPI chip-select pin
# ------------------------------------------------------------
#
# GPIO 8 is being used for the AS5047P chip-select signal.
#
# The AS5047P shares the ODrive SPI clock/data lines.
# The CS line tells the ODrive which encoder to communicate
# with.

ENCODER_CS_GPIO = 8

axis.encoder.config.abs_spi_cs_gpio_pin = ENCODER_CS_GPIO


# ------------------------------------------------------------
# Encoder resolution
# ------------------------------------------------------------
#
# AS5047P:
#
#     14-bit resolution
#
# Therefore:
#
#     2^14 = 16384 counts/revolution

ENCODER_CPR = 2**14

axis.encoder.config.cpr = ENCODER_CPR


# ------------------------------------------------------------
# Encoder bandwidth
# ------------------------------------------------------------
#
# This controls how quickly the encoder feedback is filtered.
#
# Start with a moderate value and tune later if necessary.

ENCODER_BANDWIDTH = 1000

axis.encoder.config.bandwidth = ENCODER_BANDWIDTH


print()
print("Encoder configuration:")
print(f"  Encoder: AS5047P")
print(f"  Mode: SPI absolute")
print(f"  SPI CS GPIO: {ENCODER_CS_GPIO}")
print(f"  CPR: {ENCODER_CPR}")
print(f"  Bandwidth: {ENCODER_BANDWIDTH} Hz")


# ============================================================
# 5. CONTROLLER CONFIGURATION
# ============================================================

# Initial control strategy:
#
#       Torque command
#             ↓
#       ODrive controller
#             ↓
#       Eagle Power motor
#
# We are NOT initially using:
#
#       Position control
#       Velocity control
#
# Torque control is being used because the first goal is to
# determine how much controlled torque is required to overcome
# the gearbox's initial mechanical resistance/stiction.

axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL


# ============================================================
# 6. SAFETY / VELOCITY LIMIT
# ============================================================

# Even though we are using torque control, we can keep a
# conservative velocity limit during initial testing.
#
# ODrive velocity is expressed in turns/second.
#
# 5 turns/sec = 300 RPM at the motor.

VELOCITY_LIMIT = 5.0

axis.controller.config.vel_limit = VELOCITY_LIMIT


print()
print("Controller configuration:")
print("  Control mode: TORQUE CONTROL")
print(f"  Velocity limit: {VELOCITY_LIMIT} turns/sec")


# ============================================================
# 7. CLEAR EXISTING ERRORS
# ============================================================

axis.clear_errors()

print()
print("Existing axis errors cleared.")


# ============================================================
# 8. SAVE CONFIGURATION
# ============================================================

# Save the configuration to the ODrive's non-volatile memory.
#
# This means these hardware settings remain stored on the
# ODrive after the configuration script finishes.

odrv0.save_configuration()

print()
print("ODrive configuration saved.")

print()
print("Configuration complete.")
print("Next step: motor and encoder calibration.")