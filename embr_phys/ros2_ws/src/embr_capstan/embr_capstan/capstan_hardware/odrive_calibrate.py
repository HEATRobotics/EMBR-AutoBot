"""
ODrive 3.6 (fw 0.5.6) — configure, then calibrate motor and encoder.
Eagle Power LA8308 KV130 + AS5047P (SPI), 12 V bench supply,
external 2 ohm / 50 W brake resistor.

HOW IT RUNS (two passes of the same script)
  Pass 1: if the brake resistor / DC limits / SPI encoder settings are not
          already active, the script applies them, saves, and the ODrive
          reboots. Reconnect odrivetool and run the script again.
  Pass 2: with the settings active, it runs motor calibration, then encoder
          offset calibration, then (optionally) saves the result.

It never enters closed-loop control and never commands torque or velocity.
Both calibration steps make the motor turn briefly: keep the shaft clear.

Set AXIS_INDEX to match the motor terminals (M0 -> axis 0, M1 -> axis 1).
"""

import time
import odrive

from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
    CONTROL_MODE_TORQUE_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    ENCODER_MODE_SPI_ABS_AMS,
)
from odrive.utils import dump_errors


# ============================================================
# SETTINGS
# ============================================================

AXIS_INDEX = 1                    # the axis the Eagle Power motor is wired to

# Power / brake resistor (board-wide)
DC_BUS_OVERVOLTAGE_TRIP = 15.0    # V, conservative for a 12 V supply
DC_MAX_POSITIVE_CURRENT = 4.0     # A, below the 5 A supply
DC_MAX_NEGATIVE_CURRENT = -0.01   # A, bench supply cannot absorb regen
BRAKE_RESISTANCE = 2.0            # ohm (50 W resistor, keep it mounted on metal)
MAX_REGEN_CURRENT = 0.0           # A, send regen to the resistor
ENABLE_BRAKE_RESISTOR = True      # only active after a save + reboot

# Motor
MOTOR_POLE_PAIRS = 20             # TODO: VERIFY for the LA8308 KV130
MOTOR_KV = 130
TORQUE_CONSTANT = 8.27 / MOTOR_KV # approx. 0.0636 Nm/A
CALIBRATION_CURRENT = 3.0         # A, initial test value
MOTOR_CURRENT_LIMIT = 3.0         # A, ask the team before raising

# AS5047P (14-bit, absolute) over SPI
ENCODER_CS_GPIO = 1
ENCODER_CPR = 16384               # 2^14
ENCODER_BANDWIDTH = 1000.0        # rad/s, initial starting value

# Controller (configured only; nothing is commanded here)
VELOCITY_LIMIT = 5.0              # turns/s = 300 motor RPM

# Persist the calibration so it survives a reboot. This ties the saved
# calibration to MOTOR_POLE_PAIRS, which is not yet verified. Set False to
# calibrate without saving.
SAVE_CALIBRATION = True

CALIBRATION_TIMEOUT_S = 60


# ============================================================
# HELPERS
# ============================================================

def check_errors(device, axis):
    errors = {
        "axis": axis.error,
        "motor": axis.motor.error,
        "encoder": axis.encoder.error,
        "controller": axis.controller.error,
    }
    if any(errors.values()):
        dump_errors(device)
        raise RuntimeError(f"ODrive errors detected: {errors}")


def board_config_ok(device):
    c = device.config
    return (
        bool(c.enable_brake_resistor) == ENABLE_BRAKE_RESISTOR
        and abs(c.brake_resistance - BRAKE_RESISTANCE) < 1e-6
        and abs(c.max_regen_current - MAX_REGEN_CURRENT) < 1e-6
        and abs(c.dc_max_positive_current - DC_MAX_POSITIVE_CURRENT) < 1e-3
        and abs(c.dc_max_negative_current - DC_MAX_NEGATIVE_CURRENT) < 1e-6
        and abs(c.dc_bus_overvoltage_trip_level - DC_BUS_OVERVOLTAGE_TRIP) < 1e-3
    )


def encoder_config_ok(axis):
    e = axis.encoder.config
    return (
        e.mode == ENCODER_MODE_SPI_ABS_AMS
        and e.abs_spi_cs_gpio_pin == ENCODER_CS_GPIO
        and e.cpr == ENCODER_CPR
        and not e.use_index
    )


def apply_board_settings(device):
    device.config.brake_resistance = BRAKE_RESISTANCE
    device.config.max_regen_current = MAX_REGEN_CURRENT
    device.config.enable_brake_resistor = ENABLE_BRAKE_RESISTOR
    device.config.dc_max_positive_current = DC_MAX_POSITIVE_CURRENT
    device.config.dc_max_negative_current = DC_MAX_NEGATIVE_CURRENT
    device.config.dc_bus_overvoltage_trip_level = DC_BUS_OVERVOLTAGE_TRIP


def apply_axis_settings(axis, other_axis):
    # Never start anything automatically on boot.
    for a in (axis, other_axis):
        a.config.startup_closed_loop_control = False
        a.config.startup_motor_calibration = False
        a.config.startup_encoder_index_search = False
        a.config.startup_encoder_offset_calibration = False

    # Motor
    axis.motor.config.pole_pairs = MOTOR_POLE_PAIRS
    axis.motor.config.torque_constant = TORQUE_CONSTANT
    axis.motor.config.calibration_current = CALIBRATION_CURRENT
    axis.motor.config.current_lim = MOTOR_CURRENT_LIMIT

    # AS5047P: absolute encoder, so no index search is needed.
    axis.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
    axis.encoder.config.abs_spi_cs_gpio_pin = ENCODER_CS_GPIO
    axis.encoder.config.cpr = ENCODER_CPR
    axis.encoder.config.use_index = False
    axis.encoder.config.bandwidth = ENCODER_BANDWIDTH

    # Torque control is configured, not commanded.
    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    axis.controller.config.vel_limit = VELOCITY_LIMIT
    axis.controller.config.enable_torque_mode_vel_limit = True
    axis.controller.input_torque = 0.0


def calibrate(device, axis, state, name, timeout=CALIBRATION_TIMEOUT_S):
    if axis.current_state != AXIS_STATE_IDLE:
        raise RuntimeError("Axis must be IDLE before starting calibration.")

    print(f"\nStarting {name}... (the motor may turn briefly)")
    axis.requested_state = state

    # Wait for it to start.
    start_deadline = time.monotonic() + 5
    while axis.current_state == AXIS_STATE_IDLE:
        check_errors(device, axis)
        if time.monotonic() >= start_deadline:
            raise TimeoutError(f"{name} did not start.")
        time.sleep(0.05)

    # Wait for it to finish.
    finish_deadline = time.monotonic() + timeout
    while axis.current_state != AXIS_STATE_IDLE:
        check_errors(device, axis)
        if time.monotonic() >= finish_deadline:
            raise TimeoutError(f"{name} did not finish.")
        time.sleep(0.1)

    check_errors(device, axis)
    print(f"{name} complete.")


def save_and_reboot(device):
    """save_configuration() reboots the board, so the USB connection drops.
    That disconnect is expected and is not treated as a failure."""
    try:
        device.save_configuration()
    except Exception:
        print("(Connection dropped while the ODrive rebooted. This is expected.)")


# ============================================================
# MAIN
# ============================================================

def main():
    print("Connecting to ODrive...")
    odrv0 = globals().get("dev0") or odrive.find_any(timeout=15)

    firmware = (
        odrv0.fw_version_major,
        odrv0.fw_version_minor,
        odrv0.fw_version_revision,
    )
    if firmware != (0, 5, 6):
        raise RuntimeError(
            f"This script targets firmware 0.5.6; detected {firmware}."
        )

    axis = getattr(odrv0, f"axis{AXIS_INDEX}")
    other_axis = getattr(odrv0, f"axis{1 - AXIS_INDEX}")

    print(f"Firmware 0.5.6 confirmed. Using axis {AXIS_INDEX}.")
    vbus = odrv0.vbus_voltage
    print(f"Supply voltage: {vbus:.2f} V")

    try:
        if any(a.current_state != AXIS_STATE_IDLE for a in (axis, other_axis)):
            raise RuntimeError("Both axes must be IDLE before starting.")

        if not 9.0 <= vbus <= DC_BUS_OVERVOLTAGE_TRIP - 1.0:
            raise RuntimeError(
                f"Bus voltage {vbus:.2f} V is outside the expected range "
                f"(about 12 V, well under the {DC_BUS_OVERVOLTAGE_TRIP} V trip)."
            )

        print("\nExisting errors (shown, then cleared):")
        dump_errors(odrv0)

        # ----------------------------------------------------
        # PASS 1: make sure the settings that need a reboot are active
        # ----------------------------------------------------
        if not (board_config_ok(odrv0) and encoder_config_ok(axis)):
            print("\nBrake resistor / DC limits / SPI encoder are not active yet.")
            print("Applying settings and saving. The ODrive will reboot.")
            apply_board_settings(odrv0)
            apply_axis_settings(axis, other_axis)
            save_and_reboot(odrv0)
            print("\nSaved. Reconnect odrivetool, then run this script again.")
            print("Nothing was calibrated or moved on this pass.")
            return

        # ----------------------------------------------------
        # PASS 2: calibrate
        # ----------------------------------------------------
        odrv0.clear_errors()
        check_errors(odrv0, axis)

        apply_axis_settings(axis, other_axis)   # runtime values, no reboot needed

        print("\nSettings active:")
        print(f"  Brake resistor enabled: {odrv0.config.enable_brake_resistor}")
        print(f"  Current limit / calibration current: "
              f"{axis.motor.config.current_lim} A / "
              f"{axis.motor.config.calibration_current} A")
        print(f"  Velocity limit: {axis.controller.config.vel_limit} turns/s")
        print(f"  Encoder SPI error rate: {axis.encoder.spi_error_rate}")
        print(f"  Encoder absolute position: {axis.encoder.pos_abs}")

        # Motor calibration
        calibrate(odrv0, axis, AXIS_STATE_MOTOR_CALIBRATION, "Motor calibration")
        if not axis.motor.is_calibrated:
            raise RuntimeError("Motor calibration finished but motor is not calibrated.")
        print(f"  Phase resistance: {axis.motor.config.phase_resistance:.4f} ohm")
        print(f"  Phase inductance: {axis.motor.config.phase_inductance * 1e6:.1f} uH")

        # Encoder offset calibration
        calibrate(
            odrv0, axis, AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
            "AS5047P encoder offset calibration",
        )
        if not axis.encoder.is_ready:
            raise RuntimeError("Encoder calibration finished but encoder is not ready.")
        print("AS5047P encoder is ready.")
        print(f"  Position estimate: {axis.encoder.pos_estimate}")
        print(f"  SPI error rate: {axis.encoder.spi_error_rate}")

        # Optional save
        if SAVE_CALIBRATION:
            print("\nSaving calibration (pre_calibrated = True). The ODrive will reboot.")
            axis.motor.config.pre_calibrated = True
            axis.encoder.config.pre_calibrated = True
            save_and_reboot(odrv0)
            print("\nAfter odrivetool reconnects, verify the saved calibration:")
            print(f"  dev0.axis{AXIS_INDEX}.motor.is_calibrated")
            print(f"  dev0.axis{AXIS_INDEX}.encoder.is_ready")
            print(f"  dev0.axis{AXIS_INDEX}.motor.config.pre_calibrated")
        else:
            print("\nSAVE_CALIBRATION is False: calibration is NOT saved.")

        print("\nCALIBRATION SUCCESSFUL")
        print(f"Axis {AXIS_INDEX} is IDLE. No torque or velocity was commanded.")
        print("Calibration does not prove the pole-pair count is correct.")

    except (Exception, KeyboardInterrupt):
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except Exception:
            pass
        print("\nFAILED.")
        try:
            dump_errors(odrv0)
        except Exception:
            print("(Could not read errors: the ODrive connection is not available.)")
        raise


main()