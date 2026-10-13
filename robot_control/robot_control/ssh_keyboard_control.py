import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Terminal-based teleop that works over SSH (no X display needed, unlike pynput).
# A terminal can't report key releases, so a key is treated as "held" while its
# auto-repeat keeps arriving. Before auto-repeat kicks in we wait FIRST_TIMEOUT
# (covers the OS repeat delay); once repeats arrive we switch to the much shorter
# REPEAT_TIMEOUT, so releasing a held key stops the robot almost immediately.

KEY_BINDINGS = {
    'w': (0.3, 0.3),
    's': (-0.3, -0.3),
    'a': (-0.5, 0.5),
    'd': (0.5, -0.5),
    'x': (0.0, 0.0),
}
FIRST_TIMEOUT = 0.6   # seconds; must exceed the OS initial auto-repeat delay
REPEAT_TIMEOUT = 0.2  # seconds; must exceed the auto-repeat interval + network jitter


class SshKeyboardControllerNode(Node):
    def __init__(self):
        super().__init__('ssh_keyboard_controller_node')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_commands', 10)
        self.current_L = 0.0
        self.current_R = 0.0
        self.last_key = None
        self.last_key_time = 0.0
        self.repeating = False

    def publish(self):
        msg = Float32MultiArray()
        msg.data = [self.current_L, self.current_R]
        self.publisher.publish(msg)

    def run(self):
        print(
            '\r\n========================================\r\n'
            '   Robot Keyboard Controller (SSH)\r\n'
            '----------------------------------------\r\n'
            '   W/S : Forward / Backward (hold)\r\n'
            '   A/D : Turn Left / Right  (hold)\r\n'
            '   X   : Stop\r\n'
            '   Q   : Quit\r\n'
            '========================================\r\n'
        )
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                # 20Hz loop: wait up to 0.05s for a key
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                now = self.get_clock().now().nanoseconds / 1e9
                if ready:
                    c = sys.stdin.read(1).lower()
                    if c in ('q', '\x1b'):
                        break
                    if c in KEY_BINDINGS:
                        L, R = KEY_BINDINGS[c]
                        if (L, R) != (self.current_L, self.current_R):
                            print(f'Motor command: L={L:.2f}, R={R:.2f}\r')
                        self.current_L, self.current_R = L, R
                        self.repeating = (c == self.last_key and
                                          now - self.last_key_time < FIRST_TIMEOUT)
                        self.last_key = c
                        self.last_key_time = now
                elif (self.current_L, self.current_R) != (0.0, 0.0) and \
                        now - self.last_key_time > (REPEAT_TIMEOUT if self.repeating
                                                    else FIRST_TIMEOUT):
                    self.current_L, self.current_R = 0.0, 0.0
                    self.last_key = None
                    print('Key released, stopping\r')
                self.publish()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.current_L, self.current_R = 0.0, 0.0
            self.publish()


def main(args=None):
    rclpy.init(args=args)
    node = SshKeyboardControllerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
