#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import math

class RobotMeasurer:
    def __init__(self):
        rospy.init_node('robot_measurer')
        self.joint_states = None
        self.ee_pose = None
        self.joint_received = False
        self.pose_received = False
        
        print("Starting Robot Measurer...")
        print("Looking for topics...")
        
        # Check what topics are available
        import subprocess
        try:
            result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=5)
            topics = result.stdout.strip().split('\n')
            print(f"Available topics: {len(topics)} found")
            
            joint_topics = [t for t in topics if 'joint' in t.lower()]
            pose_topics = [t for t in topics if 'pose' in t.lower()]
            
            print(f"Joint-related topics: {joint_topics}")
            print(f"Pose-related topics: {pose_topics}")
            
        except Exception as e:
            print(f"Could not list topics: {e}")
        
        print("Subscribing to topics...")
        
        # Try multiple possible topic names
        possible_joint_topics = [
            '/robot/joint_states',
            '/joint_states', 
            '/r5_dof_robot_assembly/joint_states',
            '/R5_DOF_Robot_Assembly/joint_states'
        ]
        
        possible_pose_topics = [
            '/ee_pose_from_gazebo',
            '/end_effector_pose',
            '/milestone_2_pose'
        ]
        
        # Subscribe to all possible topics
        self.joint_subs = []
        self.pose_subs = []
        
        for topic in possible_joint_topics:
            try:
                sub = rospy.Subscriber(topic, JointState, self.joint_callback, callback_args=topic)
                self.joint_subs.append(sub)
                print(f"Subscribed to joint topic: {topic}")
            except Exception as e:
                print(f"Failed to subscribe to {topic}: {e}")
                
        for topic in possible_pose_topics:
            try:
                sub = rospy.Subscriber(topic, PoseStamped, self.pose_callback, callback_args=topic)
                self.pose_subs.append(sub)
                print(f"Subscribed to pose topic: {topic}")
            except Exception as e:
                print(f"Failed to subscribe to {topic}: {e}")
        
        # Set up a timer to check status
        self.timer = rospy.Timer(rospy.Duration(2.0), self.status_check)
        
    def joint_callback(self, msg, topic_name):
        if not self.joint_received:
            print(f"✓ Received joint data from: {topic_name}")
            self.joint_received = True
        self.joint_states = msg
        self.analyze()
        
    def pose_callback(self, msg, topic_name):
        if not self.pose_received:
            print(f"✓ Received pose data from: {topic_name}")
            self.pose_received = True
        self.ee_pose = msg
        self.analyze()
        
    def status_check(self, event):
        print(f"Status: Joint data: {'✓' if self.joint_received else '✗'}, Pose data: {'✓' if self.pose_received else '✗'}")
        
        if not self.joint_received:
            print("No joint data received. Try running: rostopic echo /joint_states")
        if not self.pose_received:
            print("No pose data received. Make sure ee_pose_from_gazebo.py is running")
        
    def analyze(self):
        if self.joint_states and self.ee_pose:
            print("=" * 50)
            print("ROBOT MEASUREMENT ANALYSIS")
            print("=" * 50)
            
            # Print current joint angles
            if hasattr(self.joint_states, 'name') and hasattr(self.joint_states, 'position'):
                for i, (name, pos) in enumerate(zip(self.joint_states.name, self.joint_states.position)):
                    print(f"{name}: {math.degrees(pos):.2f}° ({pos:.4f} rad)")
            else:
                print("Joint state data format unexpected")
                
            # Print EE position
            x = self.ee_pose.pose.position.x
            y = self.ee_pose.pose.position.y
            z = self.ee_pose.pose.position.z
            
            print(f"\nEnd Effector Position:")
            print(f"X: {x:.4f}m")
            print(f"Y: {y:.4f}m") 
            print(f"Z: {z:.4f}m")
            
            # Calculate reach from base
            reach = math.sqrt(x**2 + y**2 + z**2)
            print(f"Total reach from base: {reach:.4f}m")
            
            # Calculate horizontal reach
            h_reach = math.sqrt(x**2 + y**2)
            print(f"Horizontal reach: {h_reach:.4f}m")
            
            print("=" * 50)
            print("Press Ctrl+C to exit")

if __name__ == '__main__':
    try:
        measurer = RobotMeasurer()
        print("Robot measurer started. Waiting for data...")
        print("Make sure Gazebo is running and ee_pose_from_gazebo.py is active")
        rospy.spin()
    except KeyboardInterrupt:
        print("\nShutting down robot measurer...")
    except Exception as e:
        print(f"Error: {e}")