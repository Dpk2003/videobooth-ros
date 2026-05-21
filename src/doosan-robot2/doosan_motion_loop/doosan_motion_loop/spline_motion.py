#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dsr_msgs2.srv import MoveSplineTask
from std_msgs.msg import Float64MultiArray
import time
import threading

class SplineMotionNode(Node):
    def __init__(self):
        super().__init__('spline_motion_node')

        self.client = self.create_client(MoveSplineTask, '/dsr01/motion/move_spline_task')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_spline_task...')

        self.get_logger().info('Service ready! Starting spline loop...')

        self.motion_thread = threading.Thread(target=self.motion_loop)
        self.motion_thread.daemon = True
        self.motion_thread.start()

    def make_point(self, x, y, z, a, b, c):
        point = Float64MultiArray()
        point.data = [x, y, z, a, b, c]
        return point

    def motion_loop(self):
        loop_count = 0
        forward = True
        time.sleep(1.0)

        # Forward path: Segment 1 to Segment 5
        forward_pos = [
            ( 19.810,  182.400, 859.600,  85.30, 91.75,  0.14),  # Segment 1
            (-36.680,  841.990, 429.610,  84.58, 92.45, -0.55),  # Segment 2
            (-576.570, 548.010, 293.510,  76.81, 91.57, -0.53),  # Segment 3
            ( 399.620, 635.170, 445.970,  97.34, 90.07, -0.36),  # Segment 4
            ( 50.000,  200.000, 800.000,  85.00, 91.00,  0.00),  # Segment 5
        ]

        # Reverse path: Segment 5 back to Segment 1
        reverse_pos = list(reversed(forward_pos))

        while rclpy.ok():
            direction = "forward" if forward else "reverse"
            self.get_logger().info(
                f'Sending spline loop {loop_count + 1} ({direction})...'
            )

            req          = MoveSplineTask.Request()
            pts          = forward_pos if forward else reverse_pos
            req.pos      = [self.make_point(*p) for p in pts]
            req.pos_cnt  = 5
            req.vel      = [100.0, 100.0]
            req.acc      = [200.0, 200.0]
            req.time     = 0.0
            req.mode     = 0     # absolute
            req.sync_type = 0    # blocking

            # Thread-safe way to wait for response
            event         = threading.Event()
            result_holder = [None]

            def callback(future, e=event, r=result_holder):
                r[0] = future.result()
                e.set()

            future = self.client.call_async(req)
            future.add_done_callback(callback)

            # Wait up to 60 seconds for motion to complete
            finished = event.wait(timeout=60.0)

            if not finished:
                self.get_logger().error('Motion timed out after 60s! Stopping.')
                break

            if result_holder[0] and result_holder[0].success:
                loop_count += 1
                self.get_logger().info(
                    f'Loop {loop_count} ({direction}) complete! Flipping direction...'
                )
                forward = not forward  # ping-pong direction
                time.sleep(0.5)
            else:
                self.get_logger().error('Spline motion failed! Stopping.')
                break


def main(args=None):
    rclpy.init(args=args)
    node = SplineMotionNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()