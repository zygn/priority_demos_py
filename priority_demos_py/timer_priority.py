import time 
from functools import partial
from timeit import default_timer as timer
from typing import List

import rclpy 
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(
        self,
        node_name: str = 'simple_publisher',
        topic_name: str = 'topic',
        num_topics: int = 1,
        qos_profile: int = 10,
        timer_period: float = 0.5,
        execution_time: float = 0.0
    ):
        super().__init__(node_name)

        
        self.pubs: List[Publisher] = []
        self.tmrs: List[Timer] = []
        
        for i in range(num_topics):
            pub = self.create_publisher(String, f"{topic_name}_{i+1}", qos_profile)
            tmr = self.create_timer(timer_period, partial(self.timer_callback, i+1))
            tmr.set_priority(i+1)

            self.pubs.append(pub)
            self.tmrs.append(tmr)
        
        self.execution_time = execution_time
        self.i = [0] * num_topics

    def timer_callback(self, priority: int):
        idx = priority - 1
        msg = String()
        msg.data = 'Hello, world! %d' % self.i[idx]
        # Simulate execution time
        if self.execution_time > 0.0:
            time.sleep(self.execution_time)
        self.pubs[idx].publish(msg)
        self.get_logger().info(f"Publishing: {msg.data} from `/{self.pubs[idx].topic}`")
        self.i[idx] += 1



def main(args=None):
    rclpy.init(args=args)

    pub1 = SimplePublisher(
        node_name='simple_publisher',
        qos_profile=10,
        timer_period=0.1,
        num_topics=3,
        execution_time=0.0
    )

    executor = SingleThreadedExecutor()
    executor.add_node(pub1)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        pub1.destroy_node()
        rclpy.shutdown()