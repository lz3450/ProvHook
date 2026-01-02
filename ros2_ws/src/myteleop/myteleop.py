import sys
import threading
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist

USAGE = """
This node takes keys from the keyboard and publishes them
as Twist messages.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1),
    "x": (0.9, 1),
    "e": (1, 1.1),
    "c": (1, 0.9),
}


def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(linear, angular):
    return "current velocity: \tlinear %s\tangular %s " % (linear, angular)


def main(args=None):
    settings = save_terminal_settings()

    rclpy.init(args=args)

    node = rclpy.create_node("myteleop")
    pub = node.create_publisher(Twist, "cmd_vel", 10)

    linear = 0.5
    angular = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status: int = 0

    twist_msg = Twist()

    print(USAGE)
    print(vels(linear, angular))

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    while True:
        key = get_key(settings)
        if key in MOVE_BINDINGS.keys():
            x = MOVE_BINDINGS[key][0]
            y = MOVE_BINDINGS[key][1]
            z = MOVE_BINDINGS[key][2]
            th = MOVE_BINDINGS[key][3]
        elif key in SPEED_BINDINGS.keys():
            linear = linear * SPEED_BINDINGS[key][0]
            angular = angular * SPEED_BINDINGS[key][1]

            print(vels(linear, angular))
            if status == 14:
                print(USAGE)
            status = (status + 1) % 15
        else:
            x = 0.0
            y = 0.0
            z = 0.0
            th = 0.0
            if key == "\x03":
                break

        twist_msg.linear.x = x * linear
        twist_msg.linear.y = y * linear
        twist_msg.linear.z = z * linear
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = th * angular
        pub.publish(twist_msg)

    node.destroy_node()
    rclpy.shutdown()
    spinner.join()

    restore_terminal_settings(settings)


if __name__ == "__main__":
    main()
