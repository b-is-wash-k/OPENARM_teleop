#!/usr/bin/env python3
"""
Trace joint4 values from:
1. IK output (raw)
2. ROS2 /joint_trajectory publication
3. Isaac Sim feedback (/joint_states)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import json
from datetime import datetime

class Joint4Tracer(Node):
    def __init__(self):
        super().__init__('joint4_tracer')
        
        self.sub_traj = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.traj_callback,
            10
        )
        
        self.sub_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.states_callback,
            10
        )
        
        self.count = 0
        self.log_file = open('/tmp/joint4_trace.log', 'w')
        
    def traj_callback(self, msg):
        """When IK publishes joint_trajectory"""
        self.count += 1
        
        try:
            idx_left_j4 = msg.joint_names.index('openarm_left_joint4')
            idx_right_j4 = msg.joint_names.index('openarm_right_joint4')
            
            left_val = msg.points[0].positions[idx_left_j4]
            right_val = msg.points[0].positions[idx_right_j4]
            
            log_entry = {
                'timestamp': datetime.now().isoformat(),
                'count': self.count,
                'source': 'IK → /joint_trajectory',
                'openarm_left_joint4': left_val,
                'openarm_right_joint4': right_val,
            }
            
            print(f"[{self.count:04d}] IK OUTPUT:")
            print(f"       left_joint4:  {left_val:8.4f} rad")
            print(f"       right_joint4: {right_val:8.4f} rad")
            self.log_file.write(json.dumps(log_entry) + '\n')
            self.log_file.flush()
            
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parsing trajectory: {e}")
    
    def states_callback(self, msg):
        """When Isaac Sim publishes actual joint states"""
        try:
            idx_left_j4 = msg.name.index('openarm_left_joint4')
            idx_right_j4 = msg.name.index('openarm_right_joint4')
            
            left_val = msg.position[idx_left_j4]
            right_val = msg.position[idx_right_j4]
            
            log_entry = {
                'timestamp': datetime.now().isoformat(),
                'source': 'Isaac Sim → /joint_states',
                'openarm_left_joint4': left_val,
                'openarm_right_joint4': right_val,
            }
            
            print(f"       Isaac Sim (feedback):")
            print(f"       left_joint4:  {left_val:8.4f} rad")
            print(f"       right_joint4: {right_val:8.4f} rad")
            print()
            self.log_file.write(json.dumps(log_entry) + '\n')
            self.log_file.flush()
            
        except (ValueError, IndexError) as e:
            pass  # Joint states might not have all joints

def main():
    rclpy.init()
    tracer = Joint4Tracer()
    
    print("=" * 60)
    print("JOINT4 TRACE: Monitoring IK output → Isaac Sim feedback")
    print("=" * 60)
    print()
    
    try:
        rclpy.spin(tracer)
    except KeyboardInterrupt:
        print("\n\nTrace saved to: /tmp/joint4_trace.log")
    finally:
        tracer.log_file.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
