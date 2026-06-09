#!/usr/bin/env python3

"""
Doosan A0509 - Spline Motion with Keyboard Control
==================================================

SPACE -> Run ONE complete cycle:
           Forward spline
           Reverse spline
           Return HOME
           Stop

Q     -> Emergency stop + quit

Features:
- move_spline_task smooth motion
- move_joint home return
- keyboard control
- thread-safe service calls
- safe state handling
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from dsr_msgs2.srv import (
    MoveSplineTask,
    MoveJoint,
    MoveStop
)

from std_msgs.msg import Float64MultiArray

import threading
import time
import sys
import select
import termios
import tty


# ============================================================
# CONFIG
# ============================================================

ROBOT_NS = "dsr01"

HOME_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

PRE_START_POS = [0.0, -20.0, 70.0, 0.0, 70.0, 0.0]

# Smooth cinematic speeds
SPLINE_VEL = [200.0, 200.0]
SPLINE_ACC = [400.0, 400.0]

JOINT_VEL = 120.0
JOINT_ACC = 80.0


# ============================================================
# NODE
# ============================================================

class SplineMotionNode(Node):

    def __init__(self):

        super().__init__('spline_motion_node')

        cbg = ReentrantCallbackGroup()

        # ----------------------------------------------------
        # SERVICES
        # ----------------------------------------------------

        self.spline_client = self.create_client(
            MoveSplineTask,
            f'/{ROBOT_NS}/motion/move_spline_task',
            callback_group=cbg
        )

        self.movej_client = self.create_client(
            MoveJoint,
            f'/{ROBOT_NS}/motion/move_joint',
            callback_group=cbg
        )

        self.stop_client = self.create_client(
            MoveStop,
            f'/{ROBOT_NS}/motion/move_stop',
            callback_group=cbg
        )

        self.wait_services()

        # ----------------------------------------------------
        # STATE
        # ----------------------------------------------------

        self.is_running = False
        self.should_stop = False

        # ----------------------------------------------------
        # SPLINE POINTS
        # ----------------------------------------------------

        self.forward_pos = [

            # Segment 1
            (19.810, 182.400, 859.600,
             85.30, 91.75, 0.14),

            # Segment 2
            (-36.680, 841.990, 429.610,
             84.58, 92.45, -0.55),

            # Segment 3
            (-576.570, 548.010, 293.510,
             76.81, 91.57, -0.53),

            # Segment 4
            (399.620, 635.170, 445.970,
             97.34, 90.07, -0.36),

            # Segment 5
            (50.000, 200.000, 800.000,
             85.00, 91.00, 0.00),
        ]

        self.reverse_pos = list(reversed(self.forward_pos))

        # ----------------------------------------------------
        # KEYBOARD THREAD
        # ----------------------------------------------------

        threading.Thread(
            target=self.keyboard_monitor,
            daemon=True
        ).start()

        self.print_banner()

    # ========================================================
    # WAIT SERVICES
    # ========================================================

    def wait_services(self):

        services = [

            (self.spline_client, 'move_spline_task'),
            (self.movej_client, 'move_joint'),
            (self.stop_client, 'move_stop'),

        ]

        for client, name in services:

            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    f'Waiting for {name}...'
                )

        self.get_logger().info('All services connected!')

    # ========================================================
    # THREAD SAFE SERVICE CALL
    # ========================================================

    def call_service(self, client, req, timeout=60.0):

        event = threading.Event()
        result_holder = [None]

        def callback(future):

            try:
                result_holder[0] = future.result()

            except Exception as ex:
                self.get_logger().error(
                    f'Service error: {ex}'
                )

            event.set()

        future = client.call_async(req)
        future.add_done_callback(callback)

        finished = event.wait(timeout=timeout)

        if not finished:

            self.get_logger().error(
                f'Service timeout ({timeout}s)'
            )

            return None

        return result_holder[0]

    # ========================================================
    # CREATE SPLINE POINT
    # ========================================================

    def make_point(self, x, y, z, a, b, c):

        point = Float64MultiArray()

        point.data = [
            float(x),
            float(y),
            float(z),
            float(a),
            float(b),
            float(c)
        ]

        return point

    # ========================================================
    # MOVE JOINT
    # ========================================================

    def move_home(self):

        self.get_logger().info('Moving HOME...')

        req = MoveJoint.Request()

        req.pos = HOME_POS
        req.vel = JOINT_VEL
        req.acc = JOINT_ACC

        req.time = 0.0
        req.radius = 0.0

        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0

        result = self.call_service(
            self.movej_client,
            req,
            timeout=60.0
        )

        if result and result.success:

            self.get_logger().info(
                'HOME reached!'
            )

            return True

        self.get_logger().error(
            'HOME move failed!'
        )

        return False

    # ========================================================
    # MOVE PRE START
    # ========================================================

    def move_pre_start(self):

        self.get_logger().info(
            'Moving PRE-START...'
        )

        req = MoveJoint.Request()

        req.pos = PRE_START_POS
        req.vel = JOINT_VEL
        req.acc = JOINT_ACC

        req.time = 0.0
        req.radius = 0.0

        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0

        result = self.call_service(
            self.movej_client,
            req,
            timeout=60.0
        )

        if result and result.success:

            self.get_logger().info(
                'PRE-START reached!'
            )

            return True

        self.get_logger().error(
            'PRE-START failed!'
        )

        return False

    # ========================================================
    # SPLINE MOTION
    # ========================================================

    def execute_spline(self, points, direction):

        self.get_logger().info(
            f'Running spline ({direction})...'
        )

        req = MoveSplineTask.Request()

        req.pos = [
            self.make_point(*p)
            for p in points
        ]

        req.pos_cnt = len(req.pos)

        req.vel = SPLINE_VEL
        req.acc = SPLINE_ACC

        req.time = 0.0

        req.mode = 0
        req.sync_type = 0

        result = self.call_service(
            self.spline_client,
            req,
            timeout=120.0
        )

        if result and result.success:

            self.get_logger().info(
                f'{direction} spline complete!'
            )

            return True

        self.get_logger().error(
            f'{direction} spline failed!'
        )

        return False

    # ========================================================
    # STOP ROBOT
    # ========================================================

    def stop_robot(self):

        req = MoveStop.Request()

        req.stop_mode = 1

        self.call_service(
            self.stop_client,
            req,
            timeout=5.0
        )

        self.get_logger().info(
            'Robot stopped.'
        )

    # ========================================================
    # ONE COMPLETE CYCLE
    # ========================================================

    def execute_cycle(self):

        if self.is_running:
            return

        self.is_running = True
        self.should_stop = False

        print('\n' + '=' * 50)
        print('STARTING SPLINE CYCLE')
        print('=' * 50)

        try:

            # --------------------------------------------
            # HOME
            # --------------------------------------------

            if not self.move_home():
                return

            if self.should_stop:
                return

            time.sleep(1.0)

            # --------------------------------------------
            # PRE START
            # --------------------------------------------

            if not self.move_pre_start():
                return

            if self.should_stop:
                return

            time.sleep(1.0)

            # --------------------------------------------
            # FORWARD
            # --------------------------------------------

            if not self.execute_spline(
                self.forward_pos,
                'FORWARD'
            ):
                return

            if self.should_stop:
                return

            time.sleep(0.5)

            # --------------------------------------------
            # REVERSE
            # --------------------------------------------

            if not self.execute_spline(
                self.reverse_pos,
                'REVERSE'
            ):
                return

            if self.should_stop:
                return

            time.sleep(0.5)

            # --------------------------------------------
            # RETURN HOME
            # --------------------------------------------

            self.move_home()

            print('\nCycle complete!')
            print('Press SPACE to run again.')

        finally:

            self.is_running = False

    # ========================================================
    # KEYBOARD CONTROL
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

                # ------------------------------------------------
                # SPACE -> START ONE CYCLE
                # ------------------------------------------------

                if ch == ' ':

                    if not self.is_running:

                        print('\nSPACE pressed!')
                        print('Starting cycle...\n')

                        threading.Thread(
                            target=self.execute_cycle,
                            daemon=True
                        ).start()

                    else:

                        print(
                            '\nRobot already running!\n'
                        )

                # ------------------------------------------------
                # Q -> STOP + QUIT
                # ------------------------------------------------

                elif ch.lower() == 'q':

                    print('\nStopping robot...\n')

                    self.should_stop = True

                    self.stop_robot()

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
        print('DOOSAN A0509 SPLINE MOTION')
        print('=' * 60)

        print(f'Robot Namespace : {ROBOT_NS}')
        print(f'Spline Points   : {len(self.forward_pos)}')

        print()
        print('Controls:')
        print('  SPACE -> Run one cycle')
        print('  Q     -> Stop + quit')

        print()
        print('Cycle:')
        print('  HOME')
        print('    ↓')
        print('  PRE-START')
        print('    ↓')
        print('  FORWARD SPLINE')
        print('    ↓')
        print('  REVERSE SPLINE')
        print('    ↓')
        print('  HOME')

        print('=' * 60 + '\n')


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = SplineMotionNode()

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
        except Exception:
            pass

        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
