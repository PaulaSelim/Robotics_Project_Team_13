#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import Float64     
import numpy as np

rospy.init_node('inverse_kinematics_controller', anonymous=False)  

pub1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/Joint_4/command', Float64, queue_size=10)
pub5 = rospy.Publisher('/Joint_5/command', Float64, queue_size=10)
pub6 = rospy.Publisher('/inverse_kinematics_pose', PoseStamped, queue_size=10)

print("=" * 60)
print("         INTERACTIVE INVERSE KINEMATICS CALCULATOR")
print("=" * 60)
print("Enter target coordinates to calculate joint angles")
print("Type 'quit' or 'exit' to stop")
print("=" * 60)

def inverse_kinematics(target_x, target_y, target_z):
    """
    Calculate inverse kinematics for 5-DOF robot arm
    Based on URDF joint configuration:
    
    Robot structure:
    - Joint_1: Base rotation (Z-axis)
    - Joint_2: Shoulder pitch at xyz="-2.7567E-05 0.0061999 0.04975" rpy="1.5708 0 0"
    - Joint_3: Elbow pitch at xyz="0 0.140 0.000" rpy="0 0 1.5708" 
    - Joint_4: Wrist pitch at xyz="0 0 0" rpy="-1.5708 0 -1.5708"
    - Joint_5: Wrist roll at xyz="-0.001651 -0.0054574 0.13289" rpy="1.5708 0 0"
    - EE_frame: xyz="0.007 0.1 0.013" from gripper
    
    Returns: (j1, j2, j3, j4, j5) in radians
    """
    
    # Robot link parameters from URDF (CORRECTED based on actual measurements)
    base_height = 0.04975  # Height to Joint_2 
    L1 = 0.25   # Distance from Joint_2 to Joint_3 (Arm_1 length) - CORRECTED
    L2 = 0.20   # Distance from Joint_3 to Joint_5 (Arm_2 + Arm_3) - CORRECTED  
    ee_offset = 0.02  # EE_frame offset from gripper - CORRECTED
    
    # Joint 1: Base rotation - simple atan2 for XY plane
    j1 = math.atan2(target_y, target_x)
    
    # Calculate distance from base in XY plane
    r = math.sqrt(target_x**2 + target_y**2) - ee_offset
    
    # Adjust target height for base offset
    z_adjusted = target_z - base_height
    
    # Distance from Joint_2 to target (projected)
    reach = math.sqrt(r**2 + z_adjusted**2)
    
    # Check if target is reachable
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)
    
    if reach > max_reach:
        print(f"Warning: Target unreachable. Distance {reach:.3f} > max reach {max_reach:.3f}")
        reach = max_reach * 0.95  # Scale down to reachable
    elif reach < min_reach:
        print(f"Warning: Target too close. Distance {reach:.3f} < min reach {min_reach:.3f}")
        reach = min_reach * 1.05  # Scale up to reachable
    
    # Joint 2: Shoulder angle using law of cosines
    # Angle between L1 and line to target
    try:
        cos_angle = (L1**2 + reach**2 - L2**2) / (2 * L1 * reach)
        cos_angle = max(-1, min(1, cos_angle))  # Clamp to valid range
        angle_to_target = math.atan2(z_adjusted, r)
        j2 = angle_to_target - math.acos(cos_angle)
    except:
        j2 = 0.0
        print("Warning: Could not calculate J2, using 0")
    
    # Joint 3: Elbow angle using law of cosines
    try:
        cos_j3 = (L1**2 + L2**2 - reach**2) / (2 * L1 * L2)
        cos_j3 = max(-1, min(1, cos_j3))  # Clamp to valid range
        j3 = math.pi - math.acos(cos_j3)  # Elbow angle (negative for typical "elbow up")
    except:
        j3 = 0.0
        print("Warning: Could not calculate J3, using 0")
    
    # Joint 4: Keep at 0 (as mentioned it's not used)
    j4 = 0.0
    
    # Joint 5: Wrist orientation (for now, keep end effector pointing down)
    j5 = 0.0
    
    return j1, j2, j3, j4, j5

def forward_kinematics_check(j1, j2, j3, j4, j5):
    """
    Quick forward kinematics check to verify IK solution
    Simplified version for verification
    """
    # Simplified FK for verification (CORRECTED parameters)
    base_height = 0.04975
    L1 = 0.25   # CORRECTED
    L2 = 0.20   # CORRECTED  
    ee_offset = 0.02  # CORRECTED
    
    # Calculate position
    r = L1 * math.cos(j2) + L2 * math.cos(j2 + j3) + ee_offset
    z_pos = base_height + L1 * math.sin(j2) + L2 * math.sin(j2 + j3)
    
    x_pos = r * math.cos(j1)
    y_pos = r * math.sin(j1)
    
    return x_pos, y_pos, z_pos

def get_user_input():
    """Get target coordinates from user input"""
    try:
        print("\nEnter target end effector position:")
        x_str = input("X coordinate (meters): ")
        if x_str.lower() in ['quit', 'exit', 'q']:
            return None, None, None, True
            
        y_str = input("Y coordinate (meters): ")
        if y_str.lower() in ['quit', 'exit', 'q']:
            return None, None, None, True
            
        z_str = input("Z coordinate (meters): ")
        if z_str.lower() in ['quit', 'exit', 'q']:
            return None, None, None, True
            
        x = float(x_str)
        y = float(y_str)
        z = float(z_str)
        
        return x, y, z, False
        
    except ValueError:
        print("Invalid input! Please enter numeric values.")
        return None, None, None, False
    except KeyboardInterrupt:
        print("\nExiting...")
        return None, None, None, True

def send_to_robot(j1, j2, j3, j4, j5, fk_x, fk_y, fk_z):
    """Send calculated angles to robot and publish result pose"""
    
    # Create Float64 messages
    joint_1 = Float64(j1)
    joint_2 = Float64(j2)
    joint_3 = Float64(j3)
    joint_4 = Float64(j4)
    joint_5 = Float64(j5)
    
    # Create result pose
    result_pose = PoseStamped()
    result_pose.header.frame_id = "base_link"
    result_pose.header.stamp = rospy.Time.now()
    result_pose.pose.position.x = fk_x
    result_pose.pose.position.y = fk_y
    result_pose.pose.position.z = fk_z
    
    # Set orientation to identity quaternion
    result_pose.pose.orientation.x = 0.0
    result_pose.pose.orientation.y = 0.0
    result_pose.pose.orientation.z = 0.0
    result_pose.pose.orientation.w = 1.0
    
    print("\nSending commands to robot...")
    
    # Publish joint commands
    pub1.publish(joint_1)
    pub2.publish(joint_2)
    pub3.publish(joint_3)
    pub4.publish(joint_4)
    pub5.publish(joint_5)
    
    # Publish result pose
    pub6.publish(result_pose)
    
    print("✓ Joint commands sent successfully!")

# Main interactive loop
while not rospy.is_shutdown():
    
    # Get user input
    target_x, target_y, target_z, should_exit = get_user_input()
    
    if should_exit:
        print("Goodbye!")
        break
        
    if target_x is None:  # Invalid input, try again
        continue
    
    print(f"\nTarget position: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")
    
    # Calculate inverse kinematics
    try:
        j1_val, j2_val, j3_val, j4_val, j5_val = inverse_kinematics(target_x, target_y, target_z)
        
        print(f"\nCalculated joint angles:")
        print(f"  Joint 1 = {math.degrees(j1_val):+7.2f}° ({j1_val:+.4f} rad)")
        print(f"  Joint 2 = {math.degrees(j2_val):+7.2f}° ({j2_val:+.4f} rad)")
        print(f"  Joint 3 = {math.degrees(j3_val):+7.2f}° ({j3_val:+.4f} rad)")
        print(f"  Joint 4 = {math.degrees(j4_val):+7.2f}° ({j4_val:+.4f} rad)")
        print(f"  Joint 5 = {math.degrees(j5_val):+7.2f}° ({j5_val:+.4f} rad)")
        
        # Verify with forward kinematics
        fk_x, fk_y, fk_z = forward_kinematics_check(j1_val, j2_val, j3_val, j4_val, j5_val)
        error = math.sqrt((fk_x - target_x)**2 + (fk_y - target_y)**2 + (fk_z - target_z)**2)
        
        print(f"\nForward kinematics verification:")
        print(f"  Calculated: x={fk_x:.4f}, y={fk_y:.4f}, z={fk_z:.4f}")
        print(f"  Target:     x={target_x:.4f}, y={target_y:.4f}, z={target_z:.4f}")
        print(f"  Error:      {error:.4f} m")
        
        # Ask if user wants to send to robot
        while True:
            send_cmd = input("\nSend these angles to the robot? (y/n): ").lower()
            if send_cmd in ['y', 'yes']:
                send_to_robot(j1_val, j2_val, j3_val, j4_val, j5_val, fk_x, fk_y, fk_z)
                rospy.sleep(0.1)  # Small delay to ensure messages are sent
                break
            elif send_cmd in ['n', 'no']:
                print("Command not sent.")
                break
            else:
                print("Please enter 'y' or 'n'")
        
        print("-" * 60)
        
    except Exception as e:
        print(f"❌ Inverse kinematics calculation error: {e}")
        print("-" * 60)