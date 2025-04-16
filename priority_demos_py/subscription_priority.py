import time 
from timeit import default_timer as timer
from typing import List

import rclpy 
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.subscription import Subscription
from std_msgs.msg import String



class SimpleSubscriber(Node):
    def __init__(
        self,
        node_name: str = 'simple_subscriber',
        topic_name: str = 'topic',
        num_topics: int = 1,
        qos_profile: int = 10,
        execution_time: float = 0.0
    ):
        super().__init__(node_name)

        self.subs: List[Subscription] = []

        for i in range(num_topics):
            sub = self.create_subscription(
                String,
                f"{topic_name}_{i+1}",
                self.listener_callback,
                qos_profile
            )
            sub.set_priority(i+1)
            self.subs.append(sub)

        self.execution_time = execution_time

    def listener_callback(self, msg: String):
        # Simulate execution time
        self.get_logger().info('I heard: "%s"' % msg.data)
        if self.execution_time > 0.0:
            time.sleep(self.execution_time)

def main(args=None):
    rclpy.init(args=args)

    sub1 = SimpleSubscriber(
        node_name='simple_subscriber',
        topic_name='topic',
        num_topics=3,
        qos_profile=10,
        execution_time=0.0
    )

    executor = SingleThreadedExecutor()
    executor.add_node(sub1)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        sub1.destroy_node()
        rclpy.shutdown()