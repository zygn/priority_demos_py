import time 
from timeit import default_timer as timer

import rclpy 
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String


class SimplePublisher(Node):

    def __init__(
        self,
        node_name: str = 'simple_publisher',
        topic_name: str = 'topic',
        qos_profile: int = 10,
        timer_period: float = 0.5,
        priority: int = 0,
        execution_time: float = 0.0
    ):
        super().__init__(node_name)
        self.priority = priority
        self.publisher_ = self.create_publisher(String, topic_name, qos_profile)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.execution_time = execution_time

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, world! %d' % self.i
        # Simulate execution time
        if self.execution_time > 0.0:
            time.sleep(self.execution_time)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class SimpleSubscriber(Node):
    def __init__(
        self,
        node_name: str = 'simple_subscriber',
        topic_name: str = 'topic',
        qos_profile: int = 10,
        priority: int = 0,
        execution_time: float = 0.0
    ):
        super().__init__(node_name)
        self.priority = priority
        self.subscription = self.create_subscription(
            String,
            topic_name,
            self.listener_callback,
            qos_profile)
        
        self.subscription  # prevent unused variable warning
        self.execution_time = execution_time

    def listener_callback(self, msg: String):
        # Simulate execution time
        self.get_logger().info('I heard: "%s"' % msg.data)
        if self.execution_time > 0.0:
            time.sleep(self.execution_time)


def main(args=None):
    rclpy.init(args=args)

    pub1 = SimplePublisher(
        node_name='simple_publisher1',
        topic_name='topic1',
        qos_profile=10,
        timer_period=0.01,
        priority=0,
        execution_time=0.0
    )
    pub2 = SimplePublisher(
        node_name='simple_publisher2',
        topic_name='topic2',
        qos_profile=10,
        timer_period=0.01,
        priority=1,
        execution_time=0.0
    )

    sub1 = SimpleSubscriber(
        node_name='simple_subscriber1',
        topic_name='topic1',
        qos_profile=10,
        priority=1,
        execution_time=0.0
    )
    sub2 = SimpleSubscriber(
        node_name='simple_subscriber2',
        topic_name='topic1',
        qos_profile=10,
        priority=2,
        execution_time=0.0
    )
    sub3 = SimpleSubscriber(
        node_name='simple_subscriber3',
        topic_name='topic2',
        qos_profile=10,
        priority=3,
        execution_time=0.0
    )
    sub4 = SimpleSubscriber(
        node_name='simple_subscriber4',
        topic_name='topic2',
        qos_profile=10,
        priority=4,
        execution_time=0.0
    )

    executor = SingleThreadedExecutor()
    executor.add_node(pub1)
    executor.add_node(pub2)
    executor.add_node(sub1)
    executor.add_node(sub2)
    executor.add_node(sub3)
    executor.add_node(sub4)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        pub1.destroy_node()
        pub2.destroy_node()
        sub1.destroy_node()
        sub2.destroy_node()
        sub3.destroy_node()
        sub4.destroy_node()
        rclpy.shutdown()