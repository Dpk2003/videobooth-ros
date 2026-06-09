#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.srv import MoveStop, MoveJoint, MoveLine
import threading
import time

class defaultmotion(Node):
    def __init__(self):
        super().__init__('default_motion_node')

        # Robot services
        self.stop_client = self.create_client(
            MoveStop, '/dsr01/motion/move_stop'
        )
        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_stop...')

        self.joint_client = self.create_client(
            MoveJoint, '/dsr01/motion/move_joint'
        )
        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_joint...')

        self.line_client = self.create_client(
            MoveLine, '/dsr01/motion/move_line'
        )
        while not self.line_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_line...')

        self.get_logger().info('All services ready!')
        self.get_logger().info('Press SPACE to start robot motion.')
        self.get_logger().info('Press Q to quit.')

        # State
        self.cycle_done = False
        self.running    = False

        # Start keyboard input thread
        self.input_thread = threading.Thread(
            target=self.keyboard_monitor, daemon=True
        )
        self.input_thread.start()

    # ─────────────────────────────────────────
    # KEYBOARD MONITOR
    # ─────────────────────────────────────────
    def keyboard_monitor(self):
        import sys
        import tty
        import termios

        self.get_logger().info('Keyboard monitor started.')
        self.print_status()

        fd       = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while rclpy.ok():
                ch = sys.stdin.read(1)

                # Space = start robot
                if ch == ' ':
                    if not self.running:
                        self.get_logger().info('SPACE pressed! Starting robot...')
                        self.cycle_done = False
                        self.running    = True
                        robot_thread = threading.Thread(
                            target=self.run_one_cycle, daemon=True
                        )
                        robot_thread.start()
                    else:
                        self.get_logger().info('Robot already running! Wait for cycle to finish.')

                # Q = quit
                elif ch == 'q' or ch == 'Q':
                    self.get_logger().info('Quitting...')
                    self.stop_robot()
                    rclpy.shutdown()
                    break

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def print_status(self):
        print('\n' + '='*50)
        print('  ROBOT TRIGGER NODE')
        print('='*50)
        print('  SPACE  ->  Start robot motion')
        print('  Q      ->  Quit')
        print('='*50 + '\n')

    # ─────────────────────────────────────────
    # ROBOT HELPERS
    # ─────────────────────────────────────────
    def call_async_wait(self, client, req, timeout=60.0):
        event         = threading.Event()
        result_holder = [None]

        def callback(future, e=event, r=result_holder):
            try:
                r[0] = future.result()
            except Exception as ex:
                self.get_logger().error(f'Service error: {ex}')
            e.set()

        future = client.call_async(req)
        future.add_done_callback(callback)
        finished = event.wait(timeout=timeout)

        if not finished:
            self.get_logger().error(f'Timed out after {timeout}s!')
            return None
        return result_holder[0]

    def move_to_home(self):
        self.get_logger().info('Moving to home...')
        req            = MoveJoint.Request()
        req.pos        = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        req.vel        = 40.0
        req.acc        = 40.0
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0
        result = self.call_async_wait(self.joint_client, req, timeout=60.0)
        if result and result.success:
            self.get_logger().info('Home reached!')
            time.sleep(1.0)
            return True
        self.get_logger().warn('Home move failed!')
        return False

    def move_to_pre_start(self):
        self.get_logger().info('Moving to pre-start...')
        req            = MoveJoint.Request()
        req.pos        = [0.0, -30.0, 60.0, 0.0, 60.0, 0.0]
        req.vel        = 40.0
        req.acc        = 40.0
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0
        result = self.call_async_wait(self.joint_client, req, timeout=60.0)
        if result and result.success:
            self.get_logger().info('Pre-start reached!')
            time.sleep(1.0)
            return True
        return False

    def send_line(self, x, y, z, a, b, c):
        req            = MoveLine.Request()
        req.pos        = [x, y, z, a, b, c]
        req.vel        = [100.0, 100.0]
        req.acc        = [200.0, 200.0]
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0
        return self.call_async_wait(self.line_client, req, timeout=60.0)

    def stop_robot(self):
        self.get_logger().info('Stopping robot...')
        req           = MoveStop.Request()
        req.stop_mode = 1
        self.call_async_wait(self.stop_client, req, timeout=5.0)
        self.get_logger().info('Robot stopped.')

    # ─────────────────────────────────────────
    # ROBOT MOTION - ONE CYCLE
    # ─────────────────────────────────────────
    def run_one_cycle(self):
        print('\n--- Robot cycle started ---')

        # STEP 1: Move to home
        self.move_to_home()

        # STEP 2: Move to pre-start
        self.move_to_pre_start()

        # STEP 3: Execute 5 waypoints
        waypoints = [
            ( 19.810,  182.400, 859.600,  85.30, 91.75,  0.14),  # Seg 1
            (-36.680,  841.990, 429.610,  84.58, 92.45, -0.55),  # Seg 2
            (-576.570, 548.010, 293.510,  76.81, 91.57, -0.53),  # Seg 3
            ( 399.620, 635.170, 445.970,  97.34, 90.07, -0.36),  # Seg 4
            ( 19.810,  182.400, 859.600,  85.30, 91.75,  0.14),  # Seg 5
        ]

        self.get_logger().info('Executing 5 waypoint cycle...')

        for i, (x, y, z, a, b, c) in enumerate(waypoints):
            self.get_logger().info(
                f'Waypoint {i+1}/5: ({x:.1f}, {y:.1f}, {z:.1f})'
            )
            result = self.send_line(x, y, z, a, b, c)
            if not result or not result.success:
                self.get_logger().error(f'Failed at waypoint {i+1}!')
                break
            self.get_logger().info(f'Waypoint {i+1} reached!')

        # STEP 4: Stop robot
        self.stop_robot()

        self.cycle_done = True
        self.running    = False

        print('\n--- Cycle complete! Press SPACE to run again ---\n')


def main(args=None):
    rclpy.init(args=args)
    node = defaultmotion()

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
