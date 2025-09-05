from skopt import gp_minimize
from skopt.space import Real
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import subprocess, signal, os
import time
import random


class MetricSubscriber(Node):
    def __init__(self):
        super().__init__('metric_subscriber')
        self.metric = None
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'exploration_metrics',
            self.metric_callback,
            10
        )

    def metric_callback(self, msg):
        self.metric = msg.data[0]  # Extract the area discovered / time taken metric
        print(f"metric aqquired: {self.metric}")



def evaluate_exploration_performance():
    """Subscribe to exploration_metrics to get performance."""
    rclpy.init()
    node = MetricSubscriber()

    try:
        while rclpy.ok() and node.metric is None:
            rclpy.spin_once(node, timeout_sec=1.0)

        return node.metric
    finally:
        node.destroy_node()
        rclpy.shutdown()


def objective(weights):
    w1, w2, w3 = weights
    map = ["warehouse", "maze", "depot"]
    # Launch exploration node with the current weights
    try:
        process_map = subprocess.Popen([
            'ros2', 'launch', 'turtlebot4_gz_bringup', 'turtlebot4_gz.launch.py',
            'slam:=true', 'nav2:=true', 'rviz:=true', f'world:={map[random.randint(0,2)]}'
        ], preexec_fn=os.setsid )
        process_explore = subprocess.Popen([
            'ros2', 'launch', 'frontier_exploration', 'exploration.launch.py',
            f'weight_w1:={w1}', f'weight_w2:={w2}', f'weight_w3:={w3}'
        ], preexec_fn=os.setsid )

        time.sleep(60*3)  # Run exploration for 60 seconds
    #process_explore.terminate()
    #process_map.terminate()
    finally:
        os.killpg(os.getpgid(process_map.pid), signal.SIGINT)
        process_map.wait()
        process_explore.send_signal(signal.SIGINT)
        #process_explore.wait()
    

    return -evaluate_exploration_performance()  # Return negative for maximisation


# Define parameter space
space = [
    Real(0.0, 10.0, name='w1'),
    Real(0.0, 10.0, name='w2'),
    Real(0.0, 10.0, name='w3')
]

# Perform Bayesian Optimization
res = gp_minimize(objective, space, n_calls=100, n_random_starts=10, random_state=0)

# Print the results
print(f"Best weights: w1={res.x[0]}, w2={res.x[1]}, w3={res.x[2]}")
print(f"Best performance: {-res.fun}")
