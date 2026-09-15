#!/usr/bin/env python3
"""One ESCON2 drive in profile velocity mode, using canopen-python.

Defaults to CANopen motor ID 1. Commands on motor_velocity_levels contain one
normalized motor-shaft speed, or four speeds (FL, BL, FR, BR) with only FL used.
Commission the EC-i 52 (667065) motor and motion ramps in Motion Studio first.
"""

import math
import time

import canopen
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from embr_interfaces.msg import OperationStatus
from std_msgs.msg import Float32MultiArray


class CANOpenNetwork(Node):

    def __init__(self):
        """
        ## Def:
            Create the ROS node, validate configuration, and initialize selected ESCON2
            drives in profile velocity mode. Drives are enabled on the first valid command.
            The default CANopen motor ID is 1.

        ## Args:
            N/A

        ## Returns:
            None.

        ## Raises:
            ValueError: Invalid parameters, velocity units, or requested speed limits.
            RuntimeError: A drive reports a fault or rejects profile velocity mode.
            TimeoutError: A drive state transition exceeds the configured timeout.
            CANopen/transport errors: Connection or drive initialization fails.
        """
        super().__init__('maxon_single_showcase')
        
        self.declare_parameter('interface', 'socketcan')
        self.declare_parameter('channel', 'can0')
        self.declare_parameter('bitrate', 1000000)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_period', 0.05)
        self.declare_parameter('sdo_timeout', 0.05)
        self.declare_parameter('state_timeout', 2.0)
        self.declare_parameter('max_speed_rpm', 6000.0)
        self.declare_parameter('eds_path', '')
        self.declare_parameter('showcase_motor', 1)

        self._network = canopen.Network()
        self._motors = []
        self._last_update = None
        self._fault = None
        self._closed = False
        self._status_publisher = self.create_publisher(OperationStatus, 'can_status', 10)
        try:
            self._timeout = self._positive_parameter('command_timeout')
            self._state_timeout = self._positive_parameter('state_timeout')
            period = self._positive_parameter('publish_period')
            sdo_timeout = self._positive_parameter('sdo_timeout')
            self._max_rpm = self._positive_parameter('max_speed_rpm')

            if self._max_rpm > 6000:
                raise ValueError('max_speed_rpm exceeds the 667065 mechanical limit (6000 rpm)')

            if not 1 <= self.get_parameter('showcase_motor').value <= 127:
                raise ValueError('ID must be in range [1, 127]')
            
            interface = self.get_parameter('interface').value
            channel = self.get_parameter('channel').value
            if interface in ('kvaser', 'ixxat', 'vector'):
                channel = int(channel)
            self._network.connect(interface=interface, channel=channel,
                                  bitrate=int(self._positive_parameter('bitrate')))
            
            eds = self.get_parameter('eds_path').value or None
            node_id = self.get_parameter('showcase_motor').value
            self._motor = self._network.add_node(node_id, eds)
            self._motors.append(self._motor)
            self._motor.sdo.RESPONSE_TIMEOUT = sdo_timeout
            self._motor.sdo.MAX_RETRIES = 1
                
            # Prepare drive before enabling any drive on a fresh command.
            
            self._write(self._motor, 0x6040, 0, 2)
            self._wait_state(self._motor, 0x40)
            self._write(self._motor, 0x60FF, 0, 4, signed=True)
            unit = self._read(self._motor, 0x60A9)
            if unit != 0x00B44700:
                raise ValueError(f'Node {self._motor.id}: configure velocity units as rpm (0x60A9)')
            max_motor_rpm = self._read(self._motor, 0x6080)
            max_profile_rpm = self._read(self._motor, 0x607F)
            if self._max_rpm > min(max_motor_rpm, max_profile_rpm):
                raise ValueError(
                    f'Node {self._motor.id}: requested max_speed_rpm={self._max_rpm:g} '
                    f'exceeds drive limits: 0x6080={max_motor_rpm} rpm, '
                    f'0x607F={max_profile_rpm} rpm. Set max_speed_rpm at or below '
                    f'{min(max_motor_rpm, max_profile_rpm, 6000)} rpm; '
                    'verify the commissioned drive settings in Motion Studio.')
            self._write(self._motor, 0x6060, 3, 1, signed=True)
            if self._read(self._motor, 0x6061) != 3:
                raise RuntimeError(f'Node {self._motor.id}: profile velocity mode not selected')
            self._motor.nmt.state = 'OPERATIONAL'

            self._motor_subscriber = self.create_subscription(
                Float32MultiArray, 'motor_velocity_levels', self.motor_velocity_callback,
                QoSProfile(depth=1))
            self._timer = self.create_timer(period, self._publish_frame)
            self.get_logger().info(
                f'ESCON2 node IDs {[m.id for m in self._motors]} ready; '
                'waiting for motor levels')
        except Exception:
            self._stop_all()
            self._network.disconnect()
            super().destroy_node()
            raise

    def _positive_parameter(self, name):
        """
        ## Def:
            Read a declared ROS parameter as a finite number strictly greater than zero.

        ## Args:
            `name`: String name of the declared parameter.

        ## Returns:
            Float value associated with the parameter.

        ## Raises:
            ValueError: Value is zero, negative, nonfinite, or cannot be converted to float.
            TypeError: The parameter value does not support conversion to float.
            ParameterNotDeclaredException: The named parameter has not been declared.
        """
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    @staticmethod
    def _write(motor, index, value, size, signed=False):
        """
        ## Def:
            Write an integer to object subindex 0 using a CANopen SDO download.
            Encode the value in little-endian byte order.

        ## Args:
            `motor`: CANopen remote node receiving the write.
            `index`: Integer object dictionary index, such as 0x6040.
            `value`: Value converted to an integer before encoding.
            `size`: Number of bytes used to encode the integer.
            `signed`: Whether to encode a signed integer; defaults to False.

        ## Returns:
            None.

        ## Raises:
            OverflowError: The integer does not fit the requested size and signedness.
            ValueError/TypeError: Invalid value or encoding arguments.
            CANopen/transport errors: The SDO write is rejected or communication fails.
        """
        motor.sdo.download(index, 0, int(value).to_bytes(size, 'little', signed=signed))

    @staticmethod
    def _read(motor, index):
        """
        ## Def:
            Read object subindex 0 using a CANopen SDO upload and decode its bytes
            as an unsigned little-endian integer.

        ## Args:
            `motor`: CANopen remote node to query.
            `index`: Integer object dictionary index to read.

        ## Returns:
            Unsigned integer decoded from the SDO response.

        ## Raises:
            CANopen/transport errors: The SDO read is rejected or communication fails.
        """
        return int.from_bytes(motor.sdo.upload(index, 0), 'little')

    def _wait_state(self, motor, expected):
        """
        ## Def:
            Poll the drive statusword (0x6041) until its state bits, masked with
            0x6F, match the expected state or the state timeout expires.

        ## Args:
            `motor`: CANopen remote node whose state is being checked.
            `expected`: Expected masked CiA 402 statusword state.

        ## Returns:
            None once the expected drive state is observed.

        ## Raises:
            RuntimeError: The drive statusword reports a fault.
            TimeoutError: The expected state is not reached before the deadline.
            CANopen/transport errors: Reading the statusword fails.
        """
        deadline = time.monotonic() + self._state_timeout
        while time.monotonic() < deadline:
            status = self._read(motor, 0x6041)
            if status & 0x08:
                raise RuntimeError(f'Node {motor.id}: drive fault (0x{status:04x})')
            if status & 0x6F == expected:
                return
            time.sleep(0.01)
        raise TimeoutError(f'Node {motor.id}: drive state transition timed out')

    def _stop_all(self):
        """
        ## Def:
            Attempt quick stop and a zero velocity target on every selected drive.
            Continue attempting other writes if a drive is unreachable. A successful
            write does not confirm that the motor has reached standstill.

        ## Args:
            N/A

        ## Returns:
            List of error strings for failed stop writes; empty if all writes succeed.

        ## Raises:
            None from stop writes; communication exceptions are collected in the result.
        """
        errors = []
        for motor in self._motors:
            # Quick stop uses the commissioned quick-stop option and ramp.
            for index, value, size in ((0x6040, 0x000B, 2), (0x60FF, 0, 4)):
                try:
                    self._write(motor, index, value, size)
                except Exception as exc:
                    errors.append(f'Node {motor.id}: {exc}')
        return errors

    def _fail(self, message):
        """
        ## Def:
            Clear command freshness, attempt to stop selected motors, and latch a
            failure requiring node restart. Log and publish any unconfirmed stops.

        ## Args:
            `message`: Description of the failure to log and publish.

        ## Returns:
            None.

        ## Raises:
            ROS logging/publishing errors may propagate; stop-write errors are collected.
        """
        self._last_update = None
        errors = self._stop_all()
        self._fault = message + ('; stop unconfirmed: ' + '; '.join(errors) if errors else '')
        self.get_logger().error(self._fault)
        self._publish_status(False, self._fault)

    def _publish_status(self, success, message):
        """
        ## Def:
            Publish an OperationStatus message on can_status. Success describes
            communication or readiness, not measured motor motion.

        ## Args:
            `success`: Boolean indicating whether the reported operation succeeded.
            `message`: String describing the operation or failure.

        ## Returns:
            None.

        ## Raises:
            ROS publishing errors: The status message cannot be published.
        """
        status = OperationStatus()
        status.success = success
        status.message = message
        self._status_publisher.publish(status)

    def motor_velocity_callback(self, msg):
        """
        ## Def:
            Validate normalized motor levels and send the first value as an rpm
            target to the single showcase drive.
            Enable drives on the first valid command. Invalid input or communication
            failure attempts a stop and latches a fault; later commands cannot clear it.

        ## Args:
            `msg`: Float32MultiArray with one or four finite values in [-1, 1].
                Only the first value controls the showcase motor.

        ## Returns:
            None. Publishes command acknowledgement or failure status.

        ## Raises:
            Drive and communication exceptions are handled through _fail.
            ROS logging/publishing errors may propagate while reporting a failure.
        """
        if self._fault:
            self._publish_status(False, self._fault)
            return
        if len(msg.data) not in (1, 4) or any(not math.isfinite(v) or abs(v) > 1 for v in msg.data):
            self._fail('Expected one or four finite motor levels in [-1, 1]; restart required')
            return
        received = time.monotonic()
        try:
            if self._last_update is None:
                for control, state in ((0x06, 0x21), (0x07, 0x23), (0x0F, 0x27)):
                    self._write(self._motor, 0x6040, control, 2)
                    self._wait_state(self._motor, state)
            if time.monotonic() - received >= self._timeout:
                raise TimeoutError('Command expired during drive enabling')
            for motor in self._motors:
                level = msg.data[0]
                if self._read(motor, 0x6041) & 0x6F != 0x27:
                    raise RuntimeError(f'Node {motor.id}: operation is not enabled')
                if time.monotonic() - received >= self._timeout:
                    raise TimeoutError('Command expired during CAN transfer')
                self._write(motor, 0x60FF, round(level * self._max_rpm),
                            4, signed=True)
                # ESCON2 PVM applies the target on a subsequent controlword write
                # (Application Notes, Profile Velocity Mode, steps D and E).
                if time.monotonic() - received >= self._timeout:
                    raise TimeoutError('Command expired before applying velocity target')
                self._write(motor, 0x6040, 0x000F, 2)
            self._last_update = received
            self._publish_status(True, f'{len(self._motors)} motor targets acknowledged')
        except Exception as exc:
            self._fail(f'CANopen command failed: {exc}; restart required')

    def _publish_frame(self):
        """
        ## Def:
            Run the periodic watchdog and status check. After a command, check its
            age, CAN network health, drive emergency state, and operation-enabled state.
            On failure, attempt to stop selected drives and latch a fault.

        ## Args:
            N/A

        ## Returns:
            None. Publishes readiness, operational status, or failure.

        ## Raises:
            Monitoring exceptions are handled through _fail.
            ROS logging/publishing errors may propagate while reporting status.
        """
        if self._fault:
            return
        if self._last_update is None:
            self._publish_status(True, 'Ready; waiting for motor levels')
            return
        if time.monotonic() - self._last_update > self._timeout:
            self._fail('Motor command timeout; restart required')
            return
        try:
            self._network.check()
            for motor in self._motors:
                if motor.emcy.active or self._read(motor, 0x6041) & 0x6F != 0x27:
                    raise RuntimeError(f'Node {motor.id}: drive fault or disabled')
            self._publish_status(True, f'{len(self._motors)} drives operational')
        except Exception as exc:
            self._fail(f'CANopen monitoring failed: {exc}; restart required')

    def destroy_node(self):
        """
        ## Def:
            Cancel the timer, attempt to stop selected drives, disconnect CAN, and
            release ROS node resources. Guard CAN cleanup against repeated calls.

        ## Args:
            N/A

        ## Returns:
            Boolean result from the ROS Node.destroy_node method.

        ## Raises:
            CAN disconnect or ROS cleanup errors may propagate.
            Individual stop-write errors are logged rather than raised.
        """
        if not self._closed:
            self._closed = True
            self._timer.cancel()
            errors = self._stop_all()
            if errors:
                self.get_logger().error('Shutdown stop unconfirmed: ' + '; '.join(errors))
            self._network.disconnect()
        return super().destroy_node()


def main(args=None):
    """
    ## Def:
        Initialize ROS, construct and spin the CANopen handler, and clean up on
        exit. Handle a keyboard interrupt as a normal shutdown request.

    ## Args:
        `args`: Optional ROS command-line argument list; None uses process arguments.

    ## Returns:
        None.

    ## Raises:
        Initialization, execution, or cleanup errors propagate, except KeyboardInterrupt
        raised while constructing or spinning the node.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = CANOpenNetwork()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
