#!/usr/bin/env python3

"""
Doosan A0509
Pseudo Null-Space / Gimbal Style Motion
=======================================

Goal:
- TCP appears almost fixed
- Robot joints visibly move
- Creates floating / cinematic motion

Method:
- Small TCP orbital tolerance (few mm)
- Large orientation variation
- Smooth spline interpolation

Controls:
SPACE -> Run one motion cycle
Q     -> Stop and quit
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
import math
import sys
import select
import termios
import tty


# ============================================================
# CONFIG
# ============================================================

ROBOT_NS = "dsr01"

HOME_POS = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

PRE_START_POS = [0.0, -20.0, 70.0, 0.0, 70.0, 0.0]

# Fixed visual center
CENTER_X = 350.0
CENTER_Y = 0.0
CENTER_Z = 500.0

# VERY SMALL motion radius
# creates illusion of fixed TCP
ORBIT_RADIUS = 5.0  # mm

NUM_POINTS = 40

# Smooth cinematic motion
VEL = [40.0, 40.0]
ACC = [60.0, 60.0]

JOINT_VEL = 30.0
JOINT_ACC = 30.0


# ============================================================
# NODE
# ============================================================

class GimbalMotionNode(Node):

    def __init__(self):

        super().__init__('gimbal_motion_node')

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

        self.print_banner()

        # ----------------------------------------------------
        # KEYBOARD THREAD
        # ----------------------------------------------------

        threading.Thread(
            target=self.keyboard_monitor,
            daemon=True
        ).start()

    # ========================================================
    # WAIT FOR SERVICES
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

        self.get_logger().info(
            'All services connected!'
        )

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
    # MOVE JOINT
    # ========================================================

    def move_joint(self, joints):

        req = MoveJoint.Request()

        req.pos = [float(v) for v in joints]

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

        return result and result.success

    # ========================================================
    # MAKE SPLINE POINT
    # ========================================================

    def make_point(self, x, y, z, a, b, c):

        pt = Float64MultiArray()

        pt.data = [
            float(x),
            float(y),
            float(z),
            float(a),
            float(b),
            float(c)
        ]

        return pt

    # ========================================================
    # GENERATE GIMBAL MOTION
    # ========================================================

    def generate_motion_points(self):

        points = []

        for i in range(NUM_POINTS):

            t = i / (NUM_POINTS - 1)

            theta = 2.0 * math.pi * t

            # ------------------------------------------------
            # VERY SMALL POSITION MOTION
            # ------------------------------------------------

            x = CENTER_X + ORBIT_RADIUS * math.cos(theta)

            y = CENTER_Y + ORBIT_RADIUS * math.sin(theta)

            z = CENTER_Z + (
                2.0 * math.sin(theta * 2.0)
            )

            # ------------------------------------------------
            # LARGE ORIENTATION MOTION
            # ------------------------------------------------

            # Creates visible joint motion

            a = 90.0 + 25.0 * math.sin(theta)

            b = 90.0 + 20.0 * math.cos(theta)

            c = 40.0 * math.sin(theta * 1.5)

            points.append(
                (x, y, z, a, b, c)
            )

        return points

    # ========================================================
    # EXECUTE SPLINE
    # ========================================================

    def execute_spline(self, points):

        req = MoveSplineTask.Request()

        req.pos = [
            self.make_point(*p)
            for p in points
        ]

        req.pos_cnt = len(req.pos)

        req.vel = VEL
        req.acc = ACC

        req.time = 0.0

        req.mode = 0
        req.sync_type = 0

        result = self.call_service(
            self.spline_client,
            req,
            timeout=120.0
        )

        return result and result.success

    # ========================================================
    # STOP
    # ========================================================

    def stop_robot(self):

        req = MoveStop.Request()

        req.stop_mode = 1

        self.call_service(
            self.stop_client,
            req,
            timeout=5.0
        )

    # ========================================================
    # EXECUTE ONE CYCLE
    # ========================================================

    def execute_cycle(self):

        if self.is_running:
            return

        self.is_running = True

        self.should_stop = False

        print('\n' + '=' * 60)
        print('STARTING GIMBAL STYLE MOTION')
        print('=' * 60)

        try:

            # ------------------------------------------------
            # HOME
            # ------------------------------------------------

            print('\nMoving HOME...')

            if not self.move_joint(HOME_POS):

                print('HOME failed!')
                return

            if self.should_stop:
                return

            time.sleep(1.0)

            # ------------------------------------------------
            # PRE START
            # ------------------------------------------------

            print('Moving PRE-START...')

            if not self.move_joint(PRE_START_POS):

                print('PRE-START failed!')
                return

            if self.should_stop:
                return

            time.sleep(1.0)

            # ------------------------------------------------
            # GENERATE MOTION
            # ------------------------------------------------

            print('Generating cinematic motion...')

            points = self.generate_motion_points()

            print(
                f'Generated {len(points)} spline points'
            )

            # ------------------------------------------------
            # SPLINE
            # ------------------------------------------------

            print('Running spline motion...')

            if not self.execute_spline(points):

                print('Spline failed!')
                return

            if self.should_stop:
                return

            time.sleep(0.5)

            # ------------------------------------------------
            # RETURN HOME
            # ------------------------------------------------

            print('Returning HOME...')

            self.move_joint(HOME_POS)

            print('\nMotion complete!')
            print('Press SPACE to run again.')

        finally:

            self.is_running = False

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

                # ------------------------------------------------
                # SPACE
                # ------------------------------------------------

                if ch == ' ':

                    if not self.is_running:

                        print('\nSPACE pressed')

                        threading.Thread(
                            target=self.execute_cycle,
                            daemon=True
                        ).start()

                    else:

                        print(
                            '\nRobot already moving!'
                        )

                # ------------------------------------------------
                # Q
                # ------------------------------------------------

                elif ch.lower() == 'q':

                    print('\nStopping robot...')

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

        print('\n' + '=' * 65)
        print('DOOSAN A0509 GIMBAL STYLE MOTION')
        print('=' * 65)

        print(f'Center Position :')
        print(
            f'  X={CENTER_X:.1f} '
            f'Y={CENTER_Y:.1f} '
            f'Z={CENTER_Z:.1f}'
        )

        print(
            f'Orbit Radius : {ORBIT_RADIUS:.1f} mm'
        )

        print(
            f'Spline Points: {NUM_POINTS}'
        )

        print()
        print('Behavior:')
        print('  TCP appears visually fixed')
        print('  Robot joints move dynamically')
        print('  Cinematic floating effect')

        print()
        print('Controls:')
        print('  SPACE -> Run motion')
        print('  Q     -> Stop + quit')

        print('=' * 65 + '\n')


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = GimbalMotionNode()

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