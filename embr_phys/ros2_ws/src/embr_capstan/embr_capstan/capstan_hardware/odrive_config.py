"""
STAGE 1 of 2 — minimal ODrive 3.6 (fw 0.5.6) configuration for the motor demo.

Eagle Power LA8308 KV130 on axis 0, AS5047P on SPI (CSn -> GPIO 1),
12 V bench supply (5 A), external 2 ohm / 50 W brake resistor.

This script only applies settings and saves/reboots. It does NOT move the
motor and does NOT clear errors. After the board reboots and odrivetool
reconnects, run demo_2_move.py.
"""

import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    ENCODER_MODE_SPI_ABS_AMS,
)
from odrive.utils import dump_errors


# ------------------------------------------------------------
# Settings
# ------------------------------------------------------------

BRAKE_RESISTANCE = 8.0            # ohm (50 W resistor; keep it mounted on metal)
MAX_REGEN_CURRENT = 0.0           # A, send regen to the resistor
ENABLE_BRAKE_RESISTOR = True      # takes effect after the reboot at the end

# Motor
MOTOR_POLE_PAIRS = 20             # TODO: VERIFY for the LA8308 KV130
MOTOR_KV = 130
TORQUE_CONSTANT = 8.27 / MOTOR_KV # approx. 0.0636 Nm/A
CALIBRATION_CURRENT = 3.0         # A
MOTOR_CURRENT_LIMIT = 3.0         # A, ask the team before raising

# AS5047P (14-bit) over SPI
ENCODER_CS_GPIO = 1
ENCODER_CPR = 16384
ENCODER_BANDWIDTH = 1000.0        # rad/s


# ------------------------------------------------------------
# Connect and check
# ------------------------------------------------------------

odrv0 = globals().get("dev0") or odrive.find_any(timeout=15)

version = (
    odrv0.fw_version_major,
    odrv0.fw_version_minor,
    odrv0.fw_version_revision,
)
if version != (0, 5, 6):
    raise RuntimeError(f"This script targets firmware 0.5.6; found {version}.")

axis = odrv0.axis0
other_axis = odrv0.axis1

if any(a.current_state != AXIS_STATE_IDLE for a in (axis, other_axis)):
    raise RuntimeError("Both axes must be IDLE before configuration.")

vbus = odrv0.vbus_voltage
print(f"Supply voltage: {vbus:.2f} V")
if vbus > DC_BUS_OVERVOLTAGE_TRIP - 1.0:
    raise RuntimeError("Bus voltage is too close to the overvoltage trip. Not saving.")

print("Existing errors (not cleared):")
dump_errors(odrv0)


# ------------------------------------------------------------
# Apply settings
# ------------------------------------------------------------

# Never start anything automatically on boot.
for a in (axis, other_axis):
    a.config.startup_closed_loop_control = False
    a.config.startup_motor_calibration = False
    a.config.startup_encoder_index_search = False
    a.config.startup_encoder_offset_calibration = False

# Power and brake resistor
odrv0.config.brake_resistance = BRAKE_RESISTANCE
odrv0.config.max_regen_current = MAX_REGEN_CURRENT
odrv0.config.enable_brake_resistor = ENABLE_BRAKE_RESISTOR
odrv0.config.dc_max_positive_current = DC_MAX_POSITIVE_CURRENT
odrv0.config.dc_max_negative_current = DC_MAX_NEGATIVE_CURRENT
odrv0.config.dc_bus_overvoltage_trip_level = DC_BUS_OVERVOLTAGE_TRIP

# Motor
axis.motor.config.pole_pairs = MOTOR_POLE_PAIRS
axis.motor.config.torque_constant = TORQUE_CONSTANT
axis.motor.config.calibration_current = CALIBRATION_CURRENT
axis.motor.config.current_lim = MOTOR_CURRENT_LIMIT

# Encoder
axis.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
axis.encoder.config.abs_spi_cs_gpio_pin = ENCODER_CS_GPIO
axis.encoder.config.cpr = ENCODER_CPR
axis.encoder.config.bandwidth = ENCODER_BANDWIDTH

print("\nApplied: brake resistor, DC limits, motor, SPI encoder.")


# ------------------------------------------------------------
# Save
# ------------------------------------------------------------

print("Saving. The ODrive will reboot and USB will disconnect.")
print("When odrivetool reconnects, check:")
print("  dev0.config.enable_brake_resistor  -> True")
print("  dev0.config.dc_bus_overvoltage_trip_level -> 15.0")
print("Then run demo_2_move.py.")

odrv0.save_configuration()