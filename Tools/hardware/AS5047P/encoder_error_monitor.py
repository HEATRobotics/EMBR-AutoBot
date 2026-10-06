"""
AS5047P Encoder Health Monitor
--------------------------------

Purpose:
    Verify that the AS5047P magnetic encoder is communicating
    reliably with the ODrive before the motor is energized.

This test does NOT command the motor and does NOT move the gearbox.

Checks:
    1. ODrive encoder communication / SPI error rate
    2. ODrive encoder error state
    3. Encoder position data
    4. Position stability while stationary

Important:
    This script cannot directly read the AS5047P's raw MAGL/MAGH
    diagnostic bits through the normal ODrive Python API.

    If MAGL/MAGH need to be directly verified, a raw SPI capture
    with a logic analyzer or additional ODrive firmware support
    is required.
"""

import time
import statistics
import odrive


# ---------------------------------------------------------
# Settings
# ---------------------------------------------------------

SAMPLE_RATE_HZ = 20
TEST_DURATION_SECONDS = 30

CPR = 16384

# 14-bit encoder resolution:
# 360 / 16384 = approximately 0.022 degrees per count
DEGREES_PER_COUNT = 360.0 / CPR


# ---------------------------------------------------------
# Connect to ODrive
# ---------------------------------------------------------

print("Connecting to ODrive...")

odrv0 = odrive.find_any()

print("Connected to ODrive.")

axis = odrv0.axis0
encoder = axis.encoder


# ---------------------------------------------------------
# Display encoder configuration
# ---------------------------------------------------------

print("\n========================================")
print("AS5047P ENCODER HEALTH MONITOR")
print("========================================")

print(f"Encoder CPR: {encoder.config.cpr}")
print(f"Encoder mode: {encoder.config.mode}")
print(f"SPI CS GPIO: {encoder.config.abs_spi_cs_gpio_pin}")

print("\nNo motor commands will be sent.")
print("The motor will NOT be energized by this script.")


# ---------------------------------------------------------
# Check configuration
# ---------------------------------------------------------

if encoder.config.cpr != CPR:
    print("\nWARNING:")
    print(
        f"Expected CPR = {CPR}, "
        f"but ODrive reports {encoder.config.cpr}"
    )

    print("Check the AS5047P SPI configuration.")


# ---------------------------------------------------------
# Collect encoder data
# ---------------------------------------------------------

positions = []
spi_error_rates = []

encoder_error_count = 0
read_failure_count = 0

start_time = time.time()
last_print_time = start_time


print("\nStarting stationary encoder test...")
print("Do NOT move the motor or gearbox.")
print("Testing for", TEST_DURATION_SECONDS, "seconds.\n")

print(
    "Time(s) | Position | SPI Error Rate | Encoder Error"
)
print("-" * 60)


while time.time() - start_time < TEST_DURATION_SECONDS:

    try:
        # Absolute encoder position reported by ODrive.
        position = encoder.pos_abs

        # Position estimate reported by ODrive.
        pos_estimate = encoder.pos_estimate

        # ODrive's SPI communication error metric.
        spi_error_rate = encoder.spi_error_rate

        # ODrive encoder error state.
        encoder_error = encoder.error

        positions.append(position)
        spi_error_rates.append(spi_error_rate)

        if encoder_error != 0:
            encoder_error_count += 1

        # Print approximately once per second.
        if time.time() - last_print_time >= 1.0:

            elapsed = time.time() - start_time

            print(
                f"{elapsed:7.1f} | "
                f"{position:8d} | "
                f"{spi_error_rate:14.6f} | "
                f"{encoder_error}"
            )

            last_print_time = time.time()

    except Exception as error:
        read_failure_count += 1

        print("\nEncoder read failure:")
        print(error)

    time.sleep(1.0 / SAMPLE_RATE_HZ)


# ---------------------------------------------------------
# Analyze results
# ---------------------------------------------------------

print("\n========================================")
print("TEST RESULTS")
print("========================================")

print(f"Samples collected: {len(positions)}")
print(f"Read failures:     {read_failure_count}")
print(f"Encoder errors:    {encoder_error_count}")


# ---------------------------------------------------------
# SPI error rate
# ---------------------------------------------------------

if spi_error_rates:

    average_spi_error = statistics.mean(spi_error_rates)
    maximum_spi_error = max(spi_error_rates)

    print("\nSPI COMMUNICATION")
    print("----------------------------------------")
    print(f"Average SPI error rate: {average_spi_error:.6f}")
    print(f"Maximum SPI error rate: {maximum_spi_error:.6f}")

    if maximum_spi_error == 0:
        print("PASS: No SPI communication errors observed.")

    elif maximum_spi_error < 0.1:
        print(
            "WARNING: Small amount of SPI communication error "
            "was observed."
        )

    else:
        print(
            "FAIL: Significant SPI communication errors observed."
        )


# ---------------------------------------------------------
# Encoder error state
# ---------------------------------------------------------

print("\nENCODER ERROR STATE")
print("----------------------------------------")

if encoder_error_count == 0:
    print("PASS: No ODrive encoder errors observed.")
else:
    print(
        f"FAIL: Encoder error detected in "
        f"{encoder_error_count} samples."
    )


# ---------------------------------------------------------
# Position stability
# ---------------------------------------------------------

if positions:

    position_min = min(positions)
    position_max = max(positions)

    position_range_counts = position_max - position_min
    position_range_degrees = (
        position_range_counts * DEGREES_PER_COUNT
    )

    print("\nSTATIONARY POSITION")
    print("----------------------------------------")

    print(f"Minimum position: {position_min}")
    print(f"Maximum position: {position_max}")

    print(
        f"Position variation: "
        f"{position_range_counts} counts"
    )

    print(
        f"Position variation: "
        f"{position_range_degrees:.4f} degrees"
    )

    if position_range_counts <= 2:
        print(
            "PASS: Encoder position is very stable "
            "while stationary."
        )

    elif position_range_counts <= 10:
        print(
            "WARNING: Small amount of position variation "
            "was observed."
        )

    else:
        print(
            "WARNING: Significant position variation "
            "was observed."
        )


# ---------------------------------------------------------
# Final interpretation
# ---------------------------------------------------------

print("\n========================================")
print("FINAL INTERPRETATION")
print("========================================")

if (
    read_failure_count == 0
    and encoder_error_count == 0
    and spi_error_rates
    and max(spi_error_rates) == 0
):
    print(
        "ENCODER COMMUNICATION LOOKS HEALTHY."
    )
    print(
        "The AS5047P is communicating with the ODrive "
        "without detected SPI errors."
    )
    print(
        "You can proceed to the next physical validation "
        "step before commanding the motor."
    )

else:
    print(
        "ENCODER COMMUNICATION NEEDS INVESTIGATION."
    )

print("\nIMPORTANT:")
print(
    "This test does NOT prove absolute angular accuracy."
)
print(
    "It verifies communication, error state, and "
    "stationary position stability."
)

print(
    "\nMAGL/MAGH NOTE:"
)
print(
    "Raw AS5047P magnetic-field diagnostic bits are "
    "not directly exposed here."
)
print(
    "If SPI errors occur, inspect magnet alignment, "
    "air gap, wiring, power, and EMI before energizing "
    "the motor."
)