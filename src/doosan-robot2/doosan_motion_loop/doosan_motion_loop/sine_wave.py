#!/usr/bin/env python3

"""
Doosan A0509 - SINE WAVE MOTION
================================

FEATURES:
1. Smooth sine-wave motion
2. RViz visualization
3. Real + Virtual robot compatible
4. Reduced jerk motion
5. Continuous sine cycle motion
6. Adjustable amplitude and wavelength
7. Keyboard control

MOTION:
Robot moves in X direction while Y follows sine wave:
Y = A * sin(kx)

SPACE -> Start
1 -> Slow
2 -> Normal
3 -> Fast
Q -> Quit
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from dsr_msgs2.srv import MoveJoint, MoveLine, MoveStop

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import threading
import time
import math
import sys
import select
import termios
import tty

# ============================================================
# CONFIGURATION
# ============================================================

ROBOT_NS = 'dsr01'

# ------------------------------------------------------------
# SINE WAVE PARAMETERS
# ------------------------------------------------------------

START_X = -300.0
END_X   =  300.0

BASE_Y = 300.0
BASE_Z = 500.0

AMPLITUDE = 120.0

NUM_WAYPOINTS = 120

# One complete sine cycle
# 2*pi = one cycle
WAVE_CYCLES = 1.0

# ------------------------------------------------------------
# TOOL ORIENTATION
# ------------------------------------------------------------

# Horizontal along Y-axis
TOOL_OA = 90.0
TOOL_OB = 90.0
TOOL_OC = 0.0

# ------------------------------------------------------------
# JOINT POSITIONS
# ------------------------------------------------------------

HOME_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

PRE_START_POS = [0.0, -10.0, 60.0, 0.0, 40.0, 0.0]

# ------------------------------------------------------------
# SPEEDS
# ------------------------------------------------------------

SPEED_PRESETS = {
    '1': ('SLOW',   30.0, 20.0),
    '2': ('NORMAL', 50.0, 30.0),
    '3': ('FAST',   80.0, 40.0),
}

DEFAULT_SPEED = '1'

# ============================================================


class SineWaveMotionNode(Node):

    def __init__(self):

        super().__init__('sine_wave_motion_node')

        cbg = ReentrantCallbackGroup()

        # ====================================================
        # SERVICES
        # ====================================================

        self.movej_client = self.create_client(
            MoveJoint,
            f'/{ROBOT_NS}/motion/move_joint',
            callback_group=cbg
        )

        self.movel_client = self.create_client(
            MoveLine,
            f'/{ROBOT_NS}/motion/move_line',
            callback_group=cbg
        )

        self.stop_client = self.create_client(
            MoveStop,
            f'/{ROBOT_NS}/motion/move_stop',
            callback_group=cbg
        )

        self.wait_services()

        # ====================================================
        # RVIZ
        # ====================================================

        self.path_pub = self.create_publisher(
            Marker,
            '/sine_wave_path',
            10
        )

        self.wp_pub = self.create_publisher(
            MarkerArray,
            '/sine_waypoints',
            10
        )

        self.cur_pub = self.create_publisher(
            Marker,
            '/sine_current',
            10
        )

        self.create_timer(2.0, self.publish_all)

        # ====================================================
        # STATE
        # ====================================================

        self.speed_key = DEFAULT_SPEED

        self.is_moving = False

        self.should_stop = False

        # Generate sine points
        self.waypoints = self.generate_sine_wave()

        time.sleep(0.5)

        self.publish_all()

        self.print_banner()

        threading.Thread(
            target=self.keyboard_monitor,
            daemon=True
        ).start()

    # ========================================================
    # WAIT SERVICES
    # ========================================================

    def wait_services(self):

        while not self.movej_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting move_joint...')

        while not self.movel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting move_line...')

        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting move_stop...')

        self.get_logger().info('All services connected')

    # ========================================================
    # SERVICE CALL
    # ========================================================

    def call_service(self, client, req, timeout=20.0):

        event = threading.Event()

        result_holder = [None]

        def callback(future):

            try:
                result_holder[0] = future.result()

            except Exception as e:
                self.get_logger().error(str(e))

            event.set()

        future = client.call_async(req)

        future.add_done_callback(callback)

        finished = event.wait(timeout)

        if not finished:

            self.get_logger().error('Service timeout')

            return None

        return result_holder[0]

    # ========================================================
    # SPEED
    # ========================================================

    def speed(self):

        return SPEED_PRESETS[self.speed_key]

    # ========================================================
    # MOVE JOINT
    # ========================================================

    def move_joint(self, joints, vel=40.0, acc=30.0):

        req = MoveJoint.Request()

        req.pos = [float(v) for v in joints]

        req.vel = vel

        req.acc = acc

        req.time = 0.0

        req.radius = 80.0

        req.mode = 0

        req.blend_type = 1

        req.sync_type = 0

        result = self.call_service(
            self.movej_client,
            req,
            timeout=60.0
        )

        if result and result.success:

            return True

        return False

    # ========================================================
    # MOVE LINE
    # ========================================================

    def move_line(self, x, y, z, a, b, c):

        name, vel, acc = self.speed()

        req = MoveLine.Request()

        req.pos = [
            float(x),
            float(y),
            float(z),
            float(a),
            float(b),
            float(c)
        ]

        req.vel = [vel, vel]

        req.acc = [acc, acc]

        req.time = 0.0

        # IMPORTANT FOR SMOOTH MOTION
        req.radius = 80.0

        req.ref = 0

        req.mode = 0

        req.blend_type = 1

        req.sync_type = 0

        result = self.call_service(
            self.movel_client,
            req,
            timeout=30.0
        )

        if result and result.success:

            return True

        return False

    # ========================================================
    # STOP
    # ========================================================

    def stop_motion(self):

        req = MoveStop.Request()

        req.stop_mode = 1

        self.call_service(
            self.stop_client,
            req,
            timeout=5.0
        )

    # ========================================================
    # GENERATE SINE WAVE
    # ========================================================

    def generate_sine_wave(self):

        waypoints = []

        print('\nGenerating Sine Wave Path\n')

        length = END_X - START_X

        for i in range(NUM_WAYPOINTS):

            t = i / (NUM_WAYPOINTS - 1)

            x = START_X + (length * t)

            angle = 2.0 * math.pi * WAVE_CYCLES * t

            y = BASE_Y + AMPLITUDE * math.sin(angle)

            z = BASE_Z

            print(
                f'WP{i+1:03d}: '
                f'X={x:.1f} '
                f'Y={y:.1f} '
                f'Z={z:.1f}'
            )

            waypoints.append((
                x,
                y,
                z,
                TOOL_OA,
                TOOL_OB,
                TOOL_OC
            ))

        return waypoints

    # ========================================================
    # EXECUTE SINE WAVE
    # ========================================================

    def execute_motion(self):

        self.is_moving = True

        self.should_stop = False

        print('\nStarting Sine Wave Motion\n')

        # ----------------------------------------------------
        # HOME
        # ----------------------------------------------------

        print('Moving HOME')

        if not self.move_joint(HOME_POS):

            self.is_moving = False

            return

        time.sleep(0.5)

        # ----------------------------------------------------
        # PRE START
        # ----------------------------------------------------

        print('Moving PRE-START')

        self.move_joint(PRE_START_POS)

        time.sleep(0.5)

        # ----------------------------------------------------
        # START POINT
        # ----------------------------------------------------

        first = self.waypoints[0]

        print('Moving to START')

        if not self.move_line(*first):

            self.is_moving = False

            return

        time.sleep(0.2)

        # ----------------------------------------------------
        # SINE MOTION
        # ----------------------------------------------------

        print('\nRunning sine wave...\n')

        while rclpy.ok() and not self.should_stop:

            for i, wp in enumerate(self.waypoints):

                if self.should_stop:
                    break

                x, y, z = wp[0], wp[1], wp[2]

                print(
                    f'WP{i+1}/{len(self.waypoints)} '
                    f'X={x:.1f} '
                    f'Y={y:.1f}'
                )

                self.publish_current(x, y, z)

                ok = self.move_line(*wp)

                if not ok:

                    print('Motion failed')

                    self.stop_motion()

                    self.is_moving = False

                    return

            # Reverse smoothly
            self.waypoints.reverse()

        # ----------------------------------------------------
        # STOP
        # ----------------------------------------------------

        self.stop_motion()

        self.move_joint(HOME_POS)

        self.is_moving = False

    # ========================================================
    # RVIZ
    # ========================================================

    def point(self, x, y, z):

        p = Point()

        p.x = x / 1000.0
        p.y = y / 1000.0
        p.z = z / 1000.0

        return p

    def marker(self, ns, mid, typ):

        m = Marker()

        m.header.frame_id = 'base_link'

        m.header.stamp = self.get_clock().now().to_msg()

        m.ns = ns

        m.id = mid

        m.type = typ

        m.action = Marker.ADD

        m.pose.orientation.w = 1.0

        return m

    def publish_path(self):

        m = self.marker(
            'sine_path',
            0,
            Marker.LINE_STRIP
        )

        m.scale.x = 0.01

        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0

        for wp in self.waypoints:

            m.points.append(
                self.point(wp[0], wp[1], wp[2])
            )

        self.path_pub.publish(m)

    def publish_waypoints(self):

        arr = MarkerArray()

        for i, wp in enumerate(self.waypoints):

            m = self.marker(
                'wps',
                i,
                Marker.SPHERE
            )

            m.pose.position = self.point(
                wp[0],
                wp[1],
                wp[2]
            )

            m.scale.x = 0.015
            m.scale.y = 0.015
            m.scale.z = 0.015

            m.color.r = 1.0
            m.color.g = 1.0
            m.color.a = 1.0

            arr.markers.append(m)

        self.wp_pub.publish(arr)

    def publish_current(self, x, y, z):

        m = self.marker(
            'current',
            0,
            Marker.SPHERE
        )

        m.pose.position = self.point(x, y, z)

        m.scale.x = 0.03
        m.scale.y = 0.03
        m.scale.z = 0.03

        m.color.r = 1.0
        m.color.a = 1.0

        self.cur_pub.publish(m)

    def publish_all(self):

        self.publish_path()

        self.publish_waypoints()

    # ========================================================
    # KEYBOARD
    # ========================================================

    def keyboard_monitor(self):

        fd = sys.stdin.fileno()

        old = termios.tcgetattr(fd)

        try:

            tty.setcbreak(fd)

            while rclpy.ok():

                r, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.1
                )

                if not r:
                    continue

                ch = sys.stdin.read(1)

                # SPACE
                if ch == ' ':

                    if not self.is_moving:

                        threading.Thread(
                            target=self.execute_motion,
                            daemon=True
                        ).start()

                # SPEED
                elif ch in ['1', '2', '3']:

                    self.speed_key = ch

                    n, v, a = self.speed()

                    print(f'\nSpeed -> {n} ({v} mm/s)\n')

                # QUIT
                elif ch.lower() == 'q':

                    print('\nStopping\n')

                    self.should_stop = True

                    self.stop_motion()

                    rclpy.shutdown()

                    break

        finally:

            termios.tcsetattr(
                fd,
                termios.TCSADRAIN,
                old
            )

    # ========================================================
    # BANNER
    # ========================================================

    def print_banner(self):

        print('\n' + '=' * 60)

        print('DOOSAN A0509 SINE WAVE MOTION')

        print('=' * 60)

        print(f'Amplitude   : {AMPLITUDE} mm')

        print(f'Waypoints   : {NUM_WAYPOINTS}')

        print(f'Wave Cycles : {WAVE_CYCLES}')

        print()

        print('SPACE -> START')

        print('1 -> SLOW')

        print('2 -> NORMAL')

        print('3 -> FAST')

        print('Q -> QUIT')

        print('=' * 60 + '\n')


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = SineWaveMotionNode()

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin,
        daemon=True
    )

    spin_thread.start()

    try:

        while rclpy.ok():

            time.sleep(0.1)

    except KeyboardInterrupt:

        pass

    finally:

        node.should_stop = True

        executor.shutdown()

        node.destroy_node()

        try:
            rclpy.shutdown()
        except:
            pass

        spin_thread.join(timeout=2.0)


if __name__ == '__main__':

    main()