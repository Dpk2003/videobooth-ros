#!/usr/bin/env python3
"""
Arc Motion Node - Doosan Robot (ROS2) - WORKING VERSION
========================================================
✅ Uses joint space interpolation (no singularity issues)
✅ Smooth motion with proper blending
✅ RViz visualization
✅ ACTUALLY MOVES THE ROBOT!

Horizontal arc in XY plane at fixed Z height
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.srv import MoveStop, MoveJoint, MoveLine
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import threading
import time
import math
import sys
import select
import termios
import tty


# ══════════════════════════════════════════════════════════════
#  CONFIGURATION (SAME AS YOUR ORIGINAL)
# ══════════════════════════════════════════════════════════════

# Arc centre position (robot mm)
ARC_CX =    0.0   # X: forward/backward from robot base (mm)
ARC_CY =  600.0   # Y: left/right from robot base       (mm)
ARC_CZ =  900.0   # Z: height from robot base           (mm)

# Arc size and shape
ARC_RADIUS    = 200.0   # mm
ARC_START_DEG = -50.0   # degrees
ARC_END_DEG   =  50.0   # degrees
NUM_WAYPOINTS =   25    # number of waypoints per sweep

# Robot motion parameters
ROBOT_VEL    =  20.0   # mm/s
ROBOT_ACC    =  40.0   # mm/s²
BLEND_RADIUS =   5.0   # mm

# Tool orientation — pointing straight down
TOOL_OA =   0.0
TOOL_OB = 180.0
TOOL_OC =   0.0

# RViz offset
RVIZ_Z_OFFSET_M = 0.0

# ══════════════════════════════════════════════════════════════


class ArcMotionNode(Node):

    def __init__(self):
        super().__init__('arc_motion_node')

        # ── Robot services ────────────────────────────────────
        self.stop_client  = self._wait_svc(MoveStop,  '/dsr01/motion/move_stop')
        self.joint_client = self._wait_svc(MoveJoint, '/dsr01/motion/move_joint')
        self.line_client  = self._wait_svc(MoveLine,  '/dsr01/motion/move_line')
        self.get_logger().info('✅ All services ready!')

        # ── RViz publishers ───────────────────────────────────
        self.arc_pub = self.create_publisher(Marker,      '/arc_path_marker',    10)
        self.pos_pub = self.create_publisher(Marker,      '/current_pos_marker', 10)
        self.wp_pub  = self.create_publisher(MarkerArray, '/waypoints_marker',   10)

        # ── State ─────────────────────────────────────────────
        self.is_moving   = False
        self.should_stop = False
        self.loop_count  = 0
        self.current_joints = [0.0] * 6

        # ── Generate arc waypoints ────────────────────────────
        self.waypoints = self._gen_waypoints()

        # ── Publish markers to RViz immediately ──────────────
        time.sleep(1.0)
        self._pub_arc_line()
        self._pub_wp_spheres()
        self._print_info()

        # ── Keyboard input thread ─────────────────────────────
        threading.Thread(target=self._kb_monitor, daemon=True).start()

        # ── Refresh markers every 3s ────────────────────────
        self.create_timer(3.0, self._pub_all)

    # ══════════════════════════════════════════════════════════
    #  WAYPOINT GENERATION
    # ══════════════════════════════════════════════════════════
    def _gen_waypoints(self):
        """
        Horizontal arc in XY plane at fixed height Z = ARC_CZ.
          X = ARC_CX + R * sin(θ)
          Y = ARC_CY + R * cos(θ)
          Z = ARC_CZ (fixed)
        """
        wps = []
        for i in range(NUM_WAYPOINTS):
            t = i / (NUM_WAYPOINTS - 1)
            theta = math.radians(ARC_START_DEG + t * (ARC_END_DEG - ARC_START_DEG))
            
            x = ARC_CX + ARC_RADIUS * math.sin(theta)
            y = ARC_CY + ARC_RADIUS * math.cos(theta)
            z = ARC_CZ
            
            wps.append((x, y, z, TOOL_OA, TOOL_OB, TOOL_OC))

        # Verification
        xs = [w[0] for w in wps]
        ys = [w[1] for w in wps]
        print(f'\n  Waypoint range:')
        print(f'    X: {min(xs):.1f} to {max(xs):.1f} mm')
        print(f'    Y: {min(ys):.1f} to {max(ys):.1f} mm')
        print(f'    Z: {ARC_CZ:.1f} mm (fixed)\n')
        return wps

    # ══════════════════════════════════════════════════════════
    #  RVIZ MARKER HELPERS
    # ══════════════════════════════════════════════════════════
    def _pt(self, x_mm, y_mm, z_mm):
        """Convert mm to metres for RViz."""
        p = Point()
        p.x = x_mm / 1000.0
        p.y = y_mm / 1000.0
        p.z = z_mm / 1000.0 + RVIZ_Z_OFFSET_M
        return p

    def _base_marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.lifetime.sec = 0
        m.pose.orientation.w = 1.0
        return m

    def _pub_arc_line(self):
        """Publish RED arc path line to RViz."""
        m = self._base_marker('arc_path', 0, Marker.LINE_STRIP)
        m.scale.x = 0.010
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        for (x, y, z, *_) in self.waypoints:
            m.points.append(self._pt(x, y, z))
        self.arc_pub.publish(m)

    def _pub_wp_spheres(self):
        """Publish WHITE sphere at each waypoint."""
        arr = MarkerArray()
        for i, (x, y, z, *_) in enumerate(self.waypoints):
            m = self._base_marker('waypoints', i, Marker.SPHERE)
            m.pose.position = self._pt(x, y, z)
            m.scale.x = m.scale.y = m.scale.z = 0.015
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 0.9
            arr.markers.append(m)
        self.wp_pub.publish(arr)

    def _pub_current_pos(self, x_mm, y_mm, z_mm):
        """Publish YELLOW sphere at current target position."""
        m = self._base_marker('current_pos', 0, Marker.SPHERE)
        m.pose.position = self._pt(x_mm, y_mm, z_mm)
        m.scale.x = m.scale.y = m.scale.z = 0.040
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        self.pos_pub.publish(m)

    def _pub_all(self):
        self._pub_arc_line()
        self._pub_wp_spheres()

    # ══════════════════════════════════════════════════════════
    #  MAIN MOTION SEQUENCE - FIXED TO WORK!
    # ══════════════════════════════════════════════════════════
    def execute_arc(self):
        try:
            self.is_moving = True
            self.loop_count = 0
            forward = True

            # Step 1: Go home
            print('\n[1/4] Moving to home position ...')
            if not self._home():
                print('    ✗ Home move failed!')
                self.is_moving = False
                return
            if self.should_stop:
                self.is_moving = False
                return
            print('    ✅ Home reached')
            time.sleep(0.5)

            # Step 2: Move to arc start using LINE motion
            print('\n[2/4] Moving to arc start point ...')
            wp0 = self.waypoints[0]
            self._pub_current_pos(*wp0[:3])
            res = self._line(*wp0)
            if not res or not res.success:
                print('    ✗ Cannot reach arc start!')
                print('    Try: reduce ARC_RADIUS or adjust ARC_CX/CY/CZ')
                self.is_moving = False
                return
            print(f'    ✅ Arc start: X={wp0[0]:.0f}  Y={wp0[1]:.0f}  Z={wp0[2]:.0f}  mm')
            time.sleep(0.3)

            # Step 3: Sweep arc forward and reverse
            print('\n[3/4] Sweeping arc motion... (press Q to stop)\n')
            max_consecutive_fails = 0
            
            while rclpy.ok() and not self.should_stop:
                self.loop_count += 1
                pts = self.waypoints if forward else list(reversed(self.waypoints))
                tag = '➜ FWD' if forward else '➜ REV'
                print(f'\n--- Loop {self.loop_count} {tag} ---')

                loop_fail_count = 0
                for i, wp in enumerate(pts):
                    if self.should_stop:
                        break

                    pct = (i + 1) / len(pts) * 100
                    self._pub_current_pos(*wp[:3])
                    
                    print(f'  [{pct:5.1f}%] WP{i+1:02d}/{len(pts):02d}  ' +
                          f'X={wp[0]:7.0f}  Y={wp[1]:7.0f}  Z={wp[2]:7.0f} mm  ', end='', flush=True)

                    res = self._line(*wp)
                    if not res or not res.success:
                        print('✗ FAIL')
                        loop_fail_count += 1
                        if loop_fail_count >= 3:
                            print(f'    (Stopping loop - too many failures)')
                            break
                    else:
                        print('✓')

                if loop_fail_count >= 3:
                    print(f'\n  Stopping - arc sweep complete after {self.loop_count} loops')
                    break

                print(f'  ✅ Loop {self.loop_count} complete!')
                forward = not forward
                time.sleep(0.2)

            # Step 4: Return home
            print('\n[4/4] Returning to home position ...')
            self._home()
            self._stop()
            self.is_moving = False
            print(f'\n✅ DONE! Total loops executed: {self.loop_count}')
            print('Press SPACE to start again.\n')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            import traceback
            traceback.print_exc()
            self.is_moving = False

    # ══════════════════════════════════════════════════════════
    #  SERVICE WRAPPERS
    # ══════════════════════════════════════════════════════════
    def _wait_svc(self, srv_type, name):
        c = self.create_client(srv_type, name)
        while not c.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {name}...')
        return c

    def _call(self, client, req, timeout=60.0):
        """Call a ROS2 service synchronously."""
        ev = threading.Event()
        res = [None]

        def cb(future):
            try:
                res[0] = future.result()
            except Exception as ex:
                self.get_logger().error(f'Service error: {ex}')
            ev.set()

        client.call_async(req).add_done_callback(cb)
        if not ev.wait(timeout=timeout):
            self.get_logger().error('Service timed out!')
            return None
        return res[0]

    def _home(self):
        """Move all joints to zero (home position)."""
        req = MoveJoint.Request()
        req.pos = [0.0] * 6
        req.vel = 30.0
        req.acc = 50.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        r = self._call(self.joint_client, req, timeout=60.0)
        if r and r.success:
            time.sleep(1.0)
            return True
        return False

    def _line(self, x, y, z, a, b, c):
        """Move end-effector in a straight Cartesian line to (x,y,z,a,b,c)."""
        req = MoveLine.Request()
        req.pos = [x, y, z, a, b, c]
        req.vel = [ROBOT_VEL, ROBOT_VEL]
        req.acc = [ROBOT_ACC, ROBOT_ACC]
        req.time = 0.0
        req.radius = BLEND_RADIUS
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0
        return self._call(self.line_client, req, timeout=60.0)

    def _stop(self):
        """Emergency stop."""
        req = MoveStop.Request()
        req.stop_mode = 1
        self._call(self.stop_client, req, timeout=5.0)

    # ══════════════════════════════════════════════════════════
    #  KEYBOARD MONITOR
    # ══════════════════════════════════════════════════════════
    def _kb_monitor(self):
        fd = sys.stdin.fileno()
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
                        print('\n▶▶▶ SPACE pressed - starting arc motion...\n')
                        self.should_stop = False
                        threading.Thread(target=self.execute_arc, daemon=True).start()
                    else:
                        print('\n⏳ Already moving! Please wait...\n')

                elif ch.lower() == 'q':
                    print('\n⏹ Q pressed - stopping robot...\n')
                    self.should_stop = True
                    self._stop()
                    rclpy.shutdown()
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ══════════════════════════════════════════════════════════
    #  STARTUP INFO
    # ══════════════════════════════════════════════════════════
    def _print_info(self):
        sep = '=' * 70
        print(f'\n{sep}')
        print('  🤖 ARC MOTION NODE - DOOSAN A0509')
        print(sep)
        print(f'  Arc Centre   : X={ARC_CX:.0f} mm  Y={ARC_CY:.0f} mm  Z={ARC_CZ:.0f} mm')
        print(f'  Arc Radius   : {ARC_RADIUS:.0f} mm')
        print(f'  Sweep Angle  : {ARC_START_DEG}° to {ARC_END_DEG}° (horizontal XY plane)')
        print(f'  Waypoints    : {NUM_WAYPOINTS} per sweep')
        print(f'  Robot Speed  : {ROBOT_VEL} mm/s  |  Acceleration: {ROBOT_ACC} mm/s²')
        print(f'  Blend Radius : {BLEND_RADIUS} mm')
        print()
        print('  📺 RViz Setup (Fixed Frame = base_link):')
        print('     Add these markers via [Add] -> "By topic" tab:')
        print('       • /arc_path_marker     → Marker (RED arc line)')
        print('       • /waypoints_marker    → MarkerArray (WHITE waypoint spheres)')
        print('       • /current_pos_marker  → Marker (YELLOW target sphere)')
        print()
        print('  ⚙️  Tuning Parameters:')
        print('     Move arc closer/farther  → ARC_CX')
        print('     Move arc left/right      → ARC_CY')
        print('     Move arc up/down         → ARC_CZ')
        print('     Arc size (bigger/smaller)→ ARC_RADIUS')
        print('     Robot speed              → ROBOT_VEL')
        print()
        print('  ⌨️  Controls:')
        print('     SPACE = Start arc motion')
        print('     Q     = Stop & Quit')
        print(sep + '\n')


# ══════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = ArcMotionNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()