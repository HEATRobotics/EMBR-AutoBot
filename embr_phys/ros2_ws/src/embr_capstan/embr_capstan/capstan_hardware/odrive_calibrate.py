import time
import odrive
from odrive.enums import (
    AXIS_STATE_IDLE,
    AXIS_STATE_MOTOR_CALIBRATION,
    AXIS_STATE_ENCODER_INDEX_SEARCH,
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
)
from odrive.utils import dump_errors


def check_errors(device, axis):
    errors = {
        "axis": axis.error,
        "motor": axis.motor.error,
        "encoder": axis.encoder.error,
        "controller": axis.controller.error,
    }

    if any(errors.values()):
        dump_errors(device)
        raise RuntimeError(f"Calibration errors: {errors}")


def calibrate(device, axis, state, name, timeout=60):
    if axis.current_state != AXIS_STATE_IDLE:
        raise RuntimeError("Axis must be idle before calibration.")

    print(f"Starting {name}...")
    axis.requested_state = state

    # The state request is asynchronous: first wait for it to start.
    start_deadline = time.monotonic() + 5
    while axis.current_state == AXIS_STATE_IDLE:
        check_errors(device, axis)
        if time.monotonic() >= start_deadline:
            raise TimeoutError(f"{name} did not start.")
        time.sleep(0.05)

    # Then wait for it to finish.
    finish_deadline = time.monotonic() + timeout
    while axis.current_state != AXIS_STATE_IDLE:
        check_errors(device, axis)
        if time.monotonic() >= finish_deadline:
            raise TimeoutError(f"{name} did not finish.")
        time.sleep(0.1)

    check_errors(device, axis)
    print(f"{name} complete.")


print("Connecting to ODrive...")
odrv0 = odrive.find_any(timeout=15)
axis = odrv0.axis0
print(f"Connected. Supply voltage: {odrv0.vbus_voltage:.2f} V")

try:
    if axis.current_state != AXIS_STATE_IDLE:
        raise RuntimeError("Stop the motor and put the axis in IDLE first.")

    # Display existing errors before clearing them.
    dump_errors(odrv0)
    odrv0.clear_errors()
    check_errors(odrv0, axis)

    calibrate(
        odrv0, axis,
        AXIS_STATE_MOTOR_CALIBRATION,
        "motor calibration",
    )

    if not axis.motor.is_calibrated:
        raise RuntimeError("Motor is not marked calibrated.")

    if axis.encoder.config.use_index:
        calibrate(
            odrv0, axis,
            AXIS_STATE_ENCODER_INDEX_SEARCH,
            "encoder index search",
        )
        if not axis.encoder.index_found:
            raise RuntimeError("Encoder index was not found.")

    calibrate(
        odrv0, axis,
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
        "encoder offset calibration",
    )

    if not axis.encoder.is_ready:
        raise RuntimeError("Encoder is not ready.")

    axis.motor.config.pre_calibrated = True

except (Exception, KeyboardInterrupt):
    # Best effort to stop the axis, even after failure/interruption.
    try:
        axis.requested_state = AXIS_STATE_IDLE
    except Exception:
        pass
    raise

print("Calibration successful. Saving configuration...")
odrv0.save_configuration()