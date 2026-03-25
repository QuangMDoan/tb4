import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from math import floor
from threading import Lock, Thread
from time import sleep

from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import *

BATTERY_CRITICAL = 0.1  # shutdown
BATTERY_LOW = 0.2  # go charge
BATTERY_HIGH = 0.8  # stop charging


class BatteryMonitor(Node):
    
    def __init__(self, lock: Lock):        
        super().__init__('battery_monitor')
        self.battery_lock = lock
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos_profile_sensor_data)

    def battery_callback(self, msg: BatteryState):
        with self.battery_lock:
            self.battery_pct = msg.percentage
            
    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()
        
def main(args=None):
    rclpy.init(args=args)


    navigator = TurtleBot4Navigator()

    # list of goal poses
    pose_near_dock = navigator.getPoseStamped([-1.0, -1.0], TurtleBot4Directions.WEST)

    pose_1 = navigator.getPoseStamped([-1.1372, -1.9099], TurtleBot4Directions.SOUTH)
    pose_2 = pose_near_dock

    goal_poses = [pose_1, pose_2]
    lock = Lock()
    battery_monitor = BatteryMonitor(lock)

    battery_pct = None
    position_index = 0

    battery_thread = Thread(target=battery_monitor.thread_function, daemon=True)
    battery_thread.start()
    
    # start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    start_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.WEST)
    navigator.setInitialPose(start_pose)

    # wait for Nav2
    navigator.waitUntilNav2Active()

    # undock
    navigator.undock()

    while True:
        with lock:
            battery_pct = battery_monitor.battery_pct

        if battery_pct is not None:
            navigator.info(f'Battery is at {(battery_pct*100):.2f}% charge')

            if battery_pct < BATTERY_CRITICAL:
                navigator.info('Battery critical low, shutting down...')
                break
            elif battery_pct < BATTERY_LOW:
                navigator.info('Docking for charge')
                navigator.startToPose(pose_near_dock)
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('Failed to dock...')
                    break                
            
                # wait until charged
                navigator.info('Charging...')
                battery_pct_prev = 0

                while battery_pct < BATTERY_HIGH:
                    sleep(15)
                    battery_pct_prev = floor(battery_pct*100)/100
                    with lock:
                        battery_pct = battery_monitor.battery_pct

                    # print charge level every time it increases a percent
                    if battery_pct > (battery_pct_prev + 0.01):
                        navigator.info(f'Battery is at {(battery_pct*100):.2f}% charge')
                
                navigator.undock() # undock and continue to next goal
            else:
                if position_index >= len(goal_poses):
                    position_index = 0

                navigator.info(f'Moving to goal pose {position_index+1}...')
                navigator.startToPose(goal_poses[position_index])
                position_index += 1

    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
