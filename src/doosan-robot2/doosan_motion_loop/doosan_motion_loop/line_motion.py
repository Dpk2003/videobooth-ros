#!/usr/bin/env python3
"""
Doosan A0509 line Motion - LIQUID SMOOTH VERSION
================================================
Uses move_spline_task for truly smooth motion.
All points sent at once as one continuous curve.
No stopping between waypoints = liquid smooth.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from dsr_msgs2.srv import MoveJoint, MoveLine, MoveStop, MoveSplineTask
from std_msgs.msg import Float64MultiArray
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

# Arc center (mm)
ARC_CX = 0.0
ARC_CY = 60.0
ARC_CZ = 700.0

ARC_RADIUS    = 500.0
ARC_START_DEG = -50.0
ARC_END_DEG   =  50.0

# For spline: fewer points needed (spline interpolates between them)
NUM_WAYPOINTS = 2    # 6-10 is ideal for spline

# Tool orientation
TOOL_OA = 90.0
TOOL_OB = 95.0
TOOL_OC = 0.0

# Home position
HOME_POS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Pre-start position
PRE_START_POS = [0.0, -20.0, 70.0, 0.0, 70.0, 0.0]

# Speed presets [vel mm/s, acc mm/s^2]
SPEED_PRESETS = {
    '1': ('SMOOTH',  100.0,  200.0),
    '2': ('NORMAL',  200.0, 4000.0),
    '3': ('FAST',    300.0, 600.0),
}

DEFAULT_SPEED = '1'

# ============================================================


class linemotion(Node):

    def __init__(self):
        super().__init__('line_motion_node')

        cbg = ReentrantCallbackGroup()

        # ────────────────────────────────────────
        # SERVICES
        # ────────────────────────────────────────
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
        self.spline_client = self.create_client(
            MoveSplineTask,
            f'/{ROBOT_NS}/motion/move_spline_task',
            callback_group=cbg
        )
        self.stop_client = self.create_client(
            MoveStop,
            f'/{ROBOT_NS}/motion/move_stop',
            callback_group=cbg
        )

        self.wait_services()

        # ────────────────────────────────────────
        # RVIZ PUBLISHERS
        # ────────────────────────────────────────
        self.arc_pub = self.create_publisher(
            Marker, '/arc_path_marker', 10
        )
        self.wp_pub = self.create_publisher(
            MarkerArray, '/waypoints_marker', 10
        )
        self.cur_pub = self.create_publisher(
            Marker, '/current_marker', 10
        )

        self.create_timer(2.0, self.publish_all)

        # ────────────────────────────────────────
        # STATE
        # ────────────────────────────────────────
        self.speed_key   = DEFAULT_SPEED
        self.is_moving   = False
        self.should_stop = False

        # Generate waypoints
        self.waypoints      = self.generate_waypoints()
        self.waypoints_rev  = list(reversed(self.waypoints))

        time.sleep(0.5)
        self.publish_all()
        self.print_banner()

        threading.Thread(
            target=self.keyboard_monitor, daemon=True
        ).start()

    # ────────────────────────────────────────────
    # WAIT SERVICES
    # ────────────────────────────────────────────
    def wait_services(self):
        for client, name in [
            (self.movej_client,  'move_joint'),
            (self.movel_client,  'move_line'),
            (self.spline_client, 'move_spline_task'),
            (self.stop_client,   'move_stop'),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name}...')
        self.get_logger().info('All services connected!')

    # ────────────────────────────────────────────
    # SERVICE CALL
    # ────────────────────────────────────────────
    def call_service(self, client, req, timeout=60.0):
        event         = threading.Event()
        result_holder = [None]

        def callback(future):
            try:
                result_holder[0] = future.result()
            except Exception as ex:
                self.get_logger().error(f'Service error: {ex}')
            event.set()

        future = client.call_async(req)
        future.add_done_callback(callback)
        finished = event.wait(timeout=timeout)

        if not finished:
            self.get_logger().error(f'Timed out ({timeout}s)!')
            return None
        return result_holder[0]

    # ────────────────────────────────────────────
    # SPEED
    # ────────────────────────────────────────────
    def get_speed(self):
        return SPEED_PRESETS[self.speed_key]

    # ────────────────────────────────────────────
    # MOVE JOINT
    # ────────────────────────────────────────────
    def move_joint(self, joints, vel=100.0, acc=40.0):
        self.get_logger().info(
            f'MoveJ: {[f"{v:.1f}" for v in joints]}'
        )
        req            = MoveJoint.Request()
        req.pos        = [float(j) for j in joints]
        req.vel        = vel
        req.acc        = acc
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0

        result = self.call_service(self.movej_client, req, timeout=60.0)
        if result and result.success:
            self.get_logger().info('MoveJ done!')
            return True
        self.get_logger().error('MoveJ failed!')
        return False

    # ────────────────────────────────────────────
    # MOVE LINE (only for approach)
    # ────────────────────────────────────────────
    def move_line(self, x, y, z, a, b, c):
        name, vel, acc = self.get_speed()
        req            = MoveLine.Request()
        req.pos        = [float(x), float(y), float(z),
                          float(a), float(b), float(c)]
        req.vel        = [vel, vel]
        req.acc        = [acc, acc]
        req.time       = 0.0
        req.radius     = 0.0
        req.ref        = 0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0

        result = self.call_service(self.movel_client, req, timeout=30.0)
        if result and result.success:
            return True
        self.get_logger().error('MoveL failed!')
        return False

    # ────────────────────────────────────────────
    # MOVE SPLINE - LIQUID SMOOTH
    # Sends all points at once as one smooth curve
    # ────────────────────────────────────────────
    def move_spline(self, waypoints):
        name, vel, acc = self.get_speed()

        self.get_logger().info(
            f'MoveSpline [{name}]: {len(waypoints)} points...'
        )

        req = MoveSplineTask.Request()

        # Convert waypoints to Float64MultiArray
        points = []
        for (x, y, z, a, b, c) in waypoints:
            pt      = Float64MultiArray()
            pt.data = [float(x), float(y), float(z),
                       float(a), float(b), float(c)]
            points.append(pt)

        req.pos      = points
        req.pos_cnt  = len(points)
        req.vel      = [vel, vel]
        req.acc      = [acc, acc]
        req.time     = 0.0
        req.mode     = 0
        req.sync_type = 0   # blocking - wait for full spline to finish

        result = self.call_service(
            self.spline_client, req, timeout=120.0
        )

        if result and result.success:
            self.get_logger().info('Spline done!')
            return True

        self.get_logger().error('Spline failed!')
        return False

    # ────────────────────────────────────────────
    # STOP
    # ────────────────────────────────────────────
    def stop_motion(self):
        req           = MoveStop.Request()
        req.stop_mode = 1
        self.call_service(self.stop_client, req, timeout=5.0)
        self.get_logger().info('Robot stopped.')

    # ────────────────────────────────────────────
    # GENERATE WAYPOINTS
    # ────────────────────────────────────────────
    def generate_waypoints(self):
        waypoints = []
        print('\nGenerated Arc Waypoints:')
        for i in range(NUM_WAYPOINTS):
            t     = i / (NUM_WAYPOINTS - 1)
            theta = math.radians(
                ARC_START_DEG + t * (ARC_END_DEG - ARC_START_DEG)
            )
            x = ARC_CX + ARC_RADIUS * math.sin(theta)
            y = ARC_CY + ARC_RADIUS * math.cos(theta)
            z = ARC_CZ
            print(f'  WP{i+1:02d}: X={x:.1f} Y={y:.1f} Z={z:.1f}')
            waypoints.append((x, y, z, TOOL_OA, TOOL_OB, TOOL_OC))
        print()
        return waypoints

    # ────────────────────────────────────────────
    # EXECUTE ARC - LIQUID SMOOTH
    # ────────────────────────────────────────────
    def execute_arc(self):
        self.is_moving   = True
        self.should_stop = False
        loop_count       = 0
        forward          = True

        print('\n' + '='*50)
        print('  LIQUID SMOOTH ARC MOTION STARTED')
        print('='*50)

        # STEP 1: Home
        print('\n[1/4] Moving to HOME...')
        if not self.move_joint(HOME_POS, vel=60.0, acc=120.0):
            print('HOME failed!')
            self.is_moving = False
            return
        time.sleep(1.5)

        if self.should_stop:
            self.is_moving = False
            return

        # STEP 2: Pre-start
        print('\n[2/4] Moving to PRE-START...')
        if not self.move_joint(PRE_START_POS, vel=60.0, acc=120.0):
            print('PRE-START failed! Continuing...')
        time.sleep(1.0)

        if self.should_stop:
            self.is_moving = False
            return

        # STEP 3: Move to arc start (precise)
        print('\n[3/4] Moving to ARC START...')
        start = self.waypoints[0]
        if not self.move_line(*start):
            print('Arc start failed!')
            self.is_moving = False
            return
        time.sleep(0.8)

        # STEP 4: Spline arc - all points sent at once
        print('\n[4/4] Running LIQUID SMOOTH spline arc...')
        print('      (Press Q to stop)\n')

        while rclpy.ok() and not self.should_stop:
            loop_count += 1
            direction = 'forward' if forward else 'reverse'
            pts       = (self.waypoints
                         if forward
                         else self.waypoints_rev)

            print(f'\n--- Loop {loop_count} ({direction}) ---')
            print(f'    Sending {len(pts)} points as ONE smooth spline...')

            # Publish all waypoint markers as moving dots
            for wp in pts:
                self.publish_current(wp[0], wp[1], wp[2])

            # ONE spline call = smooth curve through all points
            if not self.move_spline(pts):
                print('Spline failed! Switching to move_line fallback...')
                # Fallback: move_line with blending
                success = True
                for i, wp in enumerate(pts):
                    if self.should_stop:
                        break
                    is_last   = (i == len(pts) - 1)
                    use_blend = not is_last
                    req2           = MoveLine.Request()
                    req2.pos       = [float(v) for v in wp]
                    name, vel, acc = self.get_speed()
                    req2.vel       = [vel, vel]
                    req2.acc       = [acc, acc]
                    req2.time      = 0.0
                    req2.radius    = 50.0 if use_blend else 0.0
                    req2.ref       = 0
                    req2.mode      = 0
                    req2.blend_type = 0
                    req2.sync_type  = 0
                    result = self.call_service(
                        self.movel_client, req2, timeout=30.0
                    )
                    if not result or not result.success:
                        success = False
                        break
                if not success:
                    break

            print(f'    Loop {loop_count} ({direction}) complete!')
            forward = not forward
            time.sleep(0.3)

        # Return home
        print('\nReturning HOME...')
        self.move_joint(HOME_POS, vel=20.0, acc=40.0)
        self.stop_motion()
        self.is_moving = False

        print(f'\nArc complete! Total loops: {loop_count}')
        print('Press SPACE to start again.\n')

    # ────────────────────────────────────────────
    # RVIZ MARKERS
    # ────────────────────────────────────────────
    def make_marker(self, ns, mid, mtype):
        m                    = Marker()
        m.header.frame_id    = 'base_link'
        m.header.stamp       = self.get_clock().now().to_msg()
        m.ns                 = ns
        m.id                 = mid
        m.type               = mtype
        m.action             = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec       = 0
        return m

    def to_point(self, x, y, z):
        p   = Point()
        p.x = x / 1000.0
        p.y = y / 1000.0
        p.z = z / 1000.0
        return p

    def publish_arc(self):
        m         = self.make_marker('arc', 0, Marker.LINE_STRIP)
        m.scale.x = 0.008
        m.color.r = 0.0
        m.color.g = 0.5
        m.color.b = 1.0
        m.color.a = 1.0
        for wp in self.waypoints:
            m.points.append(self.to_point(wp[0], wp[1], wp[2]))
        self.arc_pub.publish(m)

    def publish_waypoints(self):
        arr = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            m               = self.make_marker('wps', i, Marker.SPHERE)
            m.pose.position = self.to_point(wp[0], wp[1], wp[2])
            m.scale.x       = 0.020
            m.scale.y       = 0.020
            m.scale.z       = 0.020
            m.color.r       = 1.0
            m.color.g       = 1.0
            m.color.b       = 0.0
            m.color.a       = 0.9
            arr.markers.append(m)
        self.wp_pub.publish(arr)

    def publish_current(self, x, y, z):
        m               = self.make_marker('cur', 0, Marker.SPHERE)
        m.pose.position = self.to_point(x, y, z)
        m.scale.x       = 0.035
        m.scale.y       = 0.035
        m.scale.z       = 0.035
        m.color.r       = 1.0
        m.color.g       = 0.0
        m.color.b       = 0.0
        m.color.a       = 1.0
        self.cur_pub.publish(m)

    def publish_all(self):
        self.publish_arc()
        self.publish_waypoints()

    # ────────────────────────────────────────────
    # KEYBOARD
    # ────────────────────────────────────────────
    def keyboard_monitor(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r:
                    continue
                ch = sys.stdin.read(1)

                if ch == ' ':
                    if not self.is_moving:
                        print('\nSPACE pressed! Starting arc...\n')
                        threading.Thread(
                            target=self.execute_arc, daemon=True
                        ).start()
                    else:
                        print('\nRobot already moving!\n')

                elif ch in ['1', '2', '3']:
                    if not self.is_moving:
                        self.speed_key  = ch
                        name, vel, acc = self.get_speed()
                        print(f'\nSpeed: {name} ({vel} mm/s)\n')
                    else:
                        print('\nCannot change speed while moving!\n')

                elif ch.lower() == 'q':
                    print('\nStopping...\n')
                    self.should_stop = True
                    self.stop_motion()
                    rclpy.shutdown()
                    break

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ────────────────────────────────────────────
    # BANNER
    # ────────────────────────────────────────────
    def print_banner(self):
        name, vel, acc = self.get_speed()
        print('\n' + '='*55)
        print('  DOOSAN A0509 LIQUID SMOOTH ARC MOTION')
        print('='*55)
        print(f'  Robot NS   : {ROBOT_NS}')
        print(f'  Arc Center : ({ARC_CX}, {ARC_CY}, {ARC_CZ}) mm')
        print(f'  Radius     : {ARC_RADIUS} mm')
        print(f'  Arc Range  : {ARC_START_DEG} to {ARC_END_DEG} deg')
        print(f'  Waypoints  : {NUM_WAYPOINTS} (spline interpolates)')
        print(f'  Speed      : {name} ({vel} mm/s)')
        print()
        print('  Motion: move_spline_task (one smooth curve!)')
        print('  Fallback: move_line with blending if spline fails')
        print()
        print('  RViz:')
        print('    /arc_path_marker  -> Blue arc line')
        print('    /waypoints_marker -> Yellow dots')
        print('    /current_marker   -> Red moving dot')
        print()
        print('  SPACE -> Start')
        print('  1     -> Smooth (30 mm/s)')
        print('  2     -> Normal (50 mm/s)')
        print('  3     -> Fast   (80 mm/s)')
        print('  Q     -> Quit')
        print('='*55 + '\n')


# ============================================================
# MAIN
# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = linemotion()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(
        target=executor.spin, daemon=True
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
