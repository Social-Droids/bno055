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


from rclpy.node import Node
import sys
from time import sleep


class Connector:
    """
    Parent class for bno055 connectors.

    This class does NOT contain protocol-specific code for UART, I2C, etc.
    """

    def __init__(self, node: Node):
        self.node = node

    def receive(self, reg_addr, length):
        data = None
        retries = 5

        while retries > 0:
            try:
                data = self.read(reg_addr, length)
                if retries != 5:
                    self.node.get_logger().info(f'Receive data OK after {5 - retries} retries!')
                break
            except Exception as e:
                self.node.get_logger().error('Communication error: %s' % e)
                retries -= 1
                self.node.get_logger().warn(f'Retries left {retries}')
                sleep(0.025)
                # raise e
        # if retries < 0:
        #     self.node.get_logger().fatal('Failed to Receive Data! Shutting down ROS node...')
        #     sys.exit(1)
        if data is None:
            while not self.connect():
                self.node.get_logger().warn('Resetting Connection!')
                sleep(0.05)

            self.node.get_logger().warn('Connection OK!')
        return data

    def transmit(self, reg_addr, length, data: bytes):
        return self.write(reg_addr, length, data)
