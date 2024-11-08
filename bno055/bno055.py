# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import sys
import threading

from bno055.connectors.i2c import I2C
from bno055.connectors.uart import UART
from bno055.error_handling.exceptions import BusOverRunException, \
    InvalidReading, TransmissionException
from bno055.params.NodeParameters import NodeParameters
from bno055.sensor.SensorService import SensorService
import rclpy
from rclpy.node import Node

import diagnostic_msgs
import diagnostic_updater


class Bno055Node(Node):
    """
    ROS2 Node for interfacing Bosch Bno055 IMU sensor.

    :param Node: ROS2 Node Class to initialize from
    :type Node: ROS2 Node
    :raises NotImplementedError: Indicates feature/function is not implemented yet.
    """

    sensor = None
    param = None
    lifetime = 0
    lock = None
    status = diagnostic_msgs.msg.DiagnosticStatus.OK
    status_msg = ''
    value = ['', '']
    update_diagnostic = None

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__('bno055')
        # Create lock object to prevent overlapping data queries
        self.lock = threading.Lock()

    def setup(self, _diagnostic_fn):
        # Initialize ROS2 Node Parameters:
        self.get_logger().warn('Setting up the IMU...')
        self.param = NodeParameters(self)
        self.update_diagnostic = _diagnostic_fn

        self.lifetime = self.param.max_lifetime.value

        # Get connector according to configured sensor connection type:
        if self.param.connection_type.value == UART.CONNECTIONTYPE_UART:
            connector = UART(self,
                             self.param.uart_baudrate.value,
                             self.param.uart_port.value,
                             self.param.uart_timeout.value)
        elif self.param.connection_type.value == I2C.CONNECTIONTYPE_I2C:
            connector = I2C(self,
                            self.param.i2c_bus.value,
                            self.param.i2c_addr.value)
        else:
            raise NotImplementedError('Unsupported connection type: '
                                      + str(self.param.connection_type.value))

        # Connect to BNO055 device:
        connect_retry = 10
        while not connector.connect() and connect_retry > 0:
            connect_retry -= 1
            self.get_logger().warn(f'Retrying connection to IMU. Retries left {connect_retry}')
        if connect_retry < 0:
            self.get_logger().fatal('Unable to connect to IMU!')
        self.get_logger().warn("IMU Connected")

        # Instantiate the sensor Service API:
        self.sensor = SensorService(self, connector, self.param)

        # configure imu
        try:
            self.sensor.configure()
        except IOError as e:
            self.get_logger().fatal(f' Configuring BNO055 failed! {e}')

        except TransmissionException as e:
            self.status_msg = f'Error during configuration: {e}'
            self.status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            self.update_diagnostic()

    def decrease_life(self):
        self.lifetime -= 1

    def restore_life(self):
        self.lifetime = self.param.max_lifetime.value

    def is_dead(self):
        return self.lifetime <= 0

    def read_data(self):
        """Periodic data_query_timer executions to retrieve sensor IMU data."""
        if self.lock.locked():
            # critical area still locked
            # that means that the previous data query is still being processed
            self.get_logger().info('Message communication in progress - skipping query cycle')
            return

        # Acquire lock before entering critical area to prevent overlapping data queries
        if not self.lock.acquire():
            self.get_logger().info('Failed to acquire lock!')
            return

        try:
            self.sensor.get_sensor_data()
        except TransmissionException as e:
            self.status_msg = f'Reason: {e}'
            self.status = diagnostic_msgs.msg.DiagnosticStatus.WARN

        except InvalidReading as e:
            self.get_logger().warn(f'Receiving Invalid data: {e}')
            self.decrease_life()
            self.status_msg = f'Invalid readings, lifetime = {self.lifetime}'
            self.status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            self.update_diagnostic()

            if self.is_dead():
                self.get_logger().info('Respawning Node! Failed to get sensor data')
                sys.exit(1)

        except BusOverRunException:
            # data not available yet, wait for next cycle | see #5
            self.get_logger().info('BusOverRunException')
            self.status_msg = 'Data not available'
            self.status = diagnostic_msgs.msg.DiagnosticStatus.OK
            self.update_diagnostic()
            return

        except ZeroDivisionError:
            # division by zero in get_sensor_data, return
            self.get_logger().info('ZeroDivisionError')
            self.status_msg = 'Division by zero'
            self.status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            self.update_diagnostic()
            return

        except Exception as e:  # noqa: B902
            self.status_msg = f'Receiving sensor data failed with {type(e).__name__}:"{e}"'
            self.status = diagnostic_msgs.msg.DiagnosticStatus.OK
            self.get_logger().info(self.status_msg)
            self.update_diagnostic()

        else:
            self.restore_life()
            self.status = diagnostic_msgs.msg.DiagnosticStatus.OK
            self.status_msg = 'Reading and publishing data'

        finally:
            self.lock.release()

    def log_calibration_status(self):
        """Periodic logging of calibration data (quality indicators)."""
        if self.lock.locked():
            # critical area still self.locked
            # that means that the previous data query is still being processed
            self.get_logger().info('Message communication in progress - skipping query cycle')
            # traceback.print_exc()
            return

        # Acquire self.lock before entering critical area to prevent overlapping data queries
        self.lock.acquire()
        try:
            # perform synchronized bself.lock:
            self.sensor.get_calib_status()
            self.get_logger().info('Read Calibration Status OK!')
        except Exception as e:  # noqa: B902
            self.get_logger().info('Receiving calibration status failed with %s:"%s"'
                                   % (type(e).__name__, e))
            # traceback.print_exc()
        finally:
            self.lock.release()

    def fill_diagnostic(self, stat: diagnostic_updater.DiagnosticStatusWrapper):
        stat.summary(self.status, self.status_msg)
        if self.value[0] != '':
            stat.add(self.value[0], self.value[1])
            self.value = ['', '']
        return stat

    def send_diagnostic(self, _status, _msg):
        self.status = _status
        self.status_msg = _msg
        self.update_diagnostic()


def main(args=None):
    try:
        """Main entry method for this ROS2 node."""
        # Initialize ROS Client Libraries (RCL) for Python:
        rclpy.init()

        # Create & initialize ROS2 node:
        node = Bno055Node()
        updater = diagnostic_updater.Updater(node)
        updater.setHardwareID('imu')
        updater.add('imu diagnostics', node.fill_diagnostic)

        node.setup(_diagnostic_fn=updater.force_update)

        # start regular sensor transmissions:
        # please be aware that frequencies around 30Hz and above might cause performance impacts:
        # https://github.com/ros2/rclpy/issues/520
        f = 1.0 / float(node.param.data_query_frequency.value)
        data_query_timer = node.create_timer(f, node.read_data)

        # start regular calibration status logging
        f = 1.0 / float(node.param.calib_status_frequency.value)
        status_timer = node.create_timer(f, node.log_calibration_status)

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received - exiting...')
        node.send_diagnostic(diagnostic_msgs.msg.DiagnosticStatus.ERROR, 'imu node killed!')
        sys.exit(0)
    finally:
        node.get_logger().info('ROS node shutdown')
        try:
            node.destroy_timer(data_query_timer)
            node.destroy_timer(status_timer)
        except UnboundLocalError:
            node.get_logger().info('No timers to shutdown')
        node.get_logger().warn(f'{node.get_name()} Node Finished!')
        node.send_diagnostic(diagnostic_msgs.msg.DiagnosticStatus.ERROR, 'imu node died!')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
