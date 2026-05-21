#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.srv import MoveStop, MoveJoint
import threading
import time
import math
import sys
import select
import termios
import tty

class HybridMotionNode(Node):
    def __init__(self):
        super().__init__('hybrid_motion_node')

        # SERVICES
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

        self.get_logger().info('All services ready!')

        # STATE
        self.is_moving = False
        self.should_stop = False

        def r2d(r):
            return round(math.degrees(r), 4)

        # YOUR WORKING JOINT POSES
        self.pose1 = [
            r2d(-1.5704),
            r2d(-0.65),
            r2d(-1.1166),
            r2d(0.0),
            r2d(-0.2),
            r2d(0.0),
        ]

        self.pose2 = [
            r2d(-0.3124),
            r2d(-0.65),
            r2d(-1.0028),
            r2d(-1.3538),
            r2d(-1.4927),
            r2d(1.7315),
        ]

        self.pose3 = [
            r2d(-2.2707),
            r2d(-0.65),
            r2d(-1.0028),
            r2d(-1.4904),
            r2d(0.8573),
            r2d(1.7315),
        ]

        self.vel = 60.0
        self.acc = 60.0

        self.print_info()

        self.kb_thread = threading.Thread(
            target=self.keyboard_monitor, daemon=True
        )
        self.kb_thread.start()

    def print_info(self):
        print('\n' + '='*65)
        print('  DOOSAN A0509 - SMOOTH CONTINUOUS MOTION')
        print('='*65)
        print()
        print('  Joint Poses (degrees):')
        print(f'    pose1: {[f"{v:.2f}" for v in self.pose1]}')
        print(f'    pose2: {[f"{v:.2f}" for v in self.pose2]}')
        print(f'    pose3: {[f"{v:.2f}" for v in self.pose3]}')
        print()
        print('  Speed: {:.1f} mm/s'.format(self.vel))
        print()
        print('='*65)
        print('  MOTION SEQUENCE (ONE CYCLE - SMOOTH):')
        print('    [1/6] HOME')
        print('    [2/6] POSE1 (smooth blend)')
        print('    [3/6] POSE2 (smooth blend)')
        print('    [4/6] POSE1 (smooth blend - revise)')
        print('    [5/6] POSE3 (smooth blend)')
        print('    [6/6] HOME')
        print()
        print('='*65)
        print('  SPACE  ->  Start one cycle')
        print('  Q      ->  Stop and quit')
        print('='*65 + '\n')

    def keyboard_monitor(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue

                ch = sys.stdin.read(1)

                if ch == ' ':
                    if not self.is_moving:
                        print('\nSPACE pressed! Starting cycle...\n')
                        self.should_stop = False
                        t = threading.Thread(
                            target=self.execute_one_cycle, daemon=True
                        )
                        t.start()
                    else:
                        print('\nRobot is moving! Wait...\n')

                elif ch.lower() == 'q':
                    print('\nStopping and quitting...\n')
                    self.should_stop = True
                    self.stop_robot()
                    rclpy.shutdown()
                    break

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def execute_one_cycle(self):
        try:
            self.is_moving = True

            # STEP 1: Home
            print('[1/6] Moving to HOME...')
            if not self.move_joint_smooth([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], blend_radius=50.0):
                print('HOME failed!')
                self.is_moving = False
                return

            if self.should_stop:
                self.is_moving = False
                return

            time.sleep(0.2)

            # STEP 2: Pose 1
            print('[2/6] Moving to POSE1 (smooth)...')
            if not self.move_joint_smooth(self.pose1, blend_radius=30.0):
                print('POSE1 failed!')
                self.is_moving = False
                return

            if self.should_stop:
                self.is_moving = False
                return

            time.sleep(0.2)

            # STEP 3: Pose 2
            print('[3/6] Moving to POSE2 (smooth)...')
            if not self.move_joint_smooth(self.pose2, blend_radius=30.0):
                print('POSE2 failed!')
                self.is_moving = False
                return

            if self.should_stop:
                self.is_moving = False
                return

            time.sleep(0.2)

            # STEP 4: Pose 1 (revise)
            print('[4/6] Moving to POSE1 (smooth - revise)...')
            if not self.move_joint_smooth(self.pose1, blend_radius=30.0):
                print('POSE1 (revise) failed!')
                self.is_moving = False
                return

            if self.should_stop:
                self.is_moving = False
                return

            time.sleep(0.2)

            # STEP 5: Pose 3
            print('[5/6] Moving to POSE3 (smooth)...')
            if not self.move_joint_smooth(self.pose3, blend_radius=30.0):
                print('POSE3 failed!')
                self.is_moving = False
                return

            if self.should_stop:
                self.is_moving = False
                return

            time.sleep(0.2)

            # STEP 6: Home
            print('[6/6] Returning to HOME...')
            if not self.move_joint_smooth([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], blend_radius=50.0):
                print('HOME return failed!')
                self.is_moving = False
                return

            self.stop_robot()
            self.is_moving = False

            print('\n✓ CYCLE COMPLETE!')
            print('Press SPACE to start another cycle.\n')

        except Exception as e:
            self.get_logger().error(f'Motion error: {e}')
            import traceback
            traceback.print_exc()
            self.is_moving = False

    def call_async_wait(self, client, req, timeout=60.0):
        event = threading.Event()
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
            self.get_logger().error(f'Timed out after {timeout}s!')
            return None
        return result_holder[0]

    def move_joint_smooth(self, joints, vel=None, acc=None, blend_radius=0.0):
        """
        Move with smooth blending between poses (continuous motion)
        
        blend_radius: 
          - 0.0 = abrupt stops (point-to-point)
          - 30-50 = smooth continuous motion
          - Higher = smoother but may overshoot
        """
        if vel is None:
            vel = self.vel
        if acc is None:
            acc = self.acc

        req = MoveJoint.Request()
        req.pos = joints
        req.vel = vel
        req.acc = acc
        req.time = 0.0
        req.radius = blend_radius  # THIS enables smooth continuous motion!
        req.mode = 0
        req.blend_type = 0
        req.sync_type = 0

        result = self.call_async_wait(self.joint_client, req, timeout=60.0)
        if result and result.success:
            self.get_logger().info(
                f'Smooth joint move done: {[f"{v:.2f}" for v in joints]}'
            )
            return True
        self.get_logger().warn('Smooth joint move failed!')
        return False

    def stop_robot(self):
        req = MoveStop.Request()
        req.stop_mode = 1
        self.call_async_wait(self.stop_client, req, timeout=5.0)
        self.get_logger().info('Robot stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = HybridMotionNode()

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