#!/usr/bin/env python3
import rospy
import subprocess
import time

def check_topics():
    print("Checking ROS topics...")
    
    try:
        # List all topics
        result = subprocess.run(['rostopic', 'list'], capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            print(f"\nFound {len(topics)} topics:")
            
            # Filter interesting topics
            joint_topics = [t for t in topics if 'joint' in t.lower()]
            pose_topics = [t for t in topics if 'pose' in t.lower()]
            command_topics = [t for t in topics if 'command' in t.lower()]
            
            print(f"\nJoint topics ({len(joint_topics)}):")
            for t in joint_topics:
                print(f"  {t}")
                
            print(f"\nPose topics ({len(pose_topics)}):")
            for t in pose_topics:
                print(f"  {t}")
                
            print(f"\nCommand topics ({len(command_topics)}):")
            for t in command_topics:
                print(f"  {t}")
                
            # Check if our expected topics exist
            expected = ['/robot/joint_states', '/ee_pose_from_gazebo', '/Joint_1/command']
            print(f"\nChecking expected topics:")
            for topic in expected:
                status = "✓" if topic in topics else "✗"
                print(f"  {status} {topic}")
                
        else:
            print("Failed to list topics")
            
    except Exception as e:
        print(f"Error checking topics: {e}")

if __name__ == '__main__':
    check_topics()