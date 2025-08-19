#!/usr/bin/env python3

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread
from pymoveit2 import MoveIt2

home = [0., 0., 0., 0., 0., 0.]
pos_2 = [0.0365,
         0.616,
         -1.0578,
         2.134,
         -0.2911,
         -1.698]

def main():
    rclpy.init()
    node = rclpy.create_node(node_name="my_robot_arm_control")
    logger = node.get_logger()
    
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(node=node, 
                      joint_names=['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint',
                                   'ur5_elbow_joint','ur5_wrist_1_joint',
                                   'ur5_wrist_2_joint','ur5_wrist_3_joint'],
                      base_link_name='ur5_base_link',
                      end_effector_name='gripper',
                      group_name='arm',
                      callback_group=callback_group                      )

    # Spin the node in background thread(s) and wait a bit for initialization
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    node.create_rate(1.0).sleep()

    logger.info("Move To Home Position")

    moveit2.move_to_configuration(home)
    moveit2.wait_until_executed()

    logger.info("Move To Position 2")

    moveit2.move_to_configuration(pos_2)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()