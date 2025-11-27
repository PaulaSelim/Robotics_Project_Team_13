#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import Float64     
import numpy as np
from sensor_msgs.msg import JointState

rospy.init_node('inverse_kinematics_controller', anonymous=False)  

pub1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/Joint_4/command', Float64, queue_size=10)
pub5 = rospy.Publisher('/Joint_5/command', Float64, queue_size=10)
pub6 = rospy.Publisher('/inverse_kinematics_pose', PoseStamped, queue_size=10)

# Constants for Newton-Raphson IK solver (from forwardKinematics.py)
DOF = 4
MAX_ITERATIONS = 200
TOLERANCE = 1e-6
DAMPING_FACTOR = 0.01

# Robot parameters (from forwardKinematics.py)
DEFAULT_L_VALUES = [0.04355, 0.140, 0.1329, 0.0]  # [L1, L2, L3, L4]
DEFAULT_A_VALUES = [0.0, -0.03786, -0.0387, 0.0]  # [a1, a2, a3, a4]

# Current joint state (updated from /joint_states)
current_joints = None
joint_name_order = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']

# Subscribe to joint_states so we can compare current position and avoid unnecessary moves
def joint_state_callback(msg):
    global current_joints
    # Try to build an ordered list of the first five joint positions matching expected names
    try:
        if hasattr(msg, 'name') and len(msg.name) > 0:
            positions = [None] * 5
            for i, jn in enumerate(joint_name_order):
                if jn in msg.name:
                    idx = msg.name.index(jn)
                    if idx < len(msg.position):
                        positions[i] = msg.position[idx]
            # If any are still None, try to fall back to the first five positions
            if any(p is None for p in positions) and len(msg.position) >= 5:
                positions = list(msg.position[:5])
            current_joints = positions
        else:
            # No names provided — use first five positions if available
            if len(msg.position) >= 5:
                current_joints = list(msg.position[:5])
            else:
                current_joints = None
    except Exception:
        current_joints = None

try:
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
except Exception:
    # subscription may fail if topic not present; continue without it
    pass

print("=" * 60)
print("         INTERACTIVE INVERSE KINEMATICS CALCULATOR")
print("=" * 60)
print("Enter target coordinates to calculate joint angles")
print("Type 'quit' or 'exit' to stop")
print("=" * 60)

def _create_dh_transformation(theta, d, a, alpha):
    """Create Denavit-Hartenberg transformation matrix using numpy"""
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)
    cos_alpha = math.cos(alpha)
    sin_alpha = math.sin(alpha)
    
    return np.array([
        [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
        [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
        [0, sin_alpha, cos_alpha, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_dh(joint_angles, l_values=None, a_values=None):
    """Calculate forward kinematics using DH parameters (numerical implementation)"""
    if l_values is None:
        l_values = DEFAULT_L_VALUES
    if a_values is None:
        a_values = DEFAULT_A_VALUES
    
    q1, q2, q3, q4 = joint_angles
    L1, L2, L3, L4 = l_values
    a1, a2, a3, a4 = a_values
    
    # DH transformation matrices (same as forwardKinematics.py)
    # Frame 0 to 1: (q1, L1, 0, pi/2)
    # Frame 1 to 2: (pi/2 + q2, 0, L2, pi)
    # Frame 2 to 3: (q3, 0, L3, pi)
    # Frame 3 to 4: (q4, 0, L4, 0)
    
    T01 = _create_dh_transformation(q1, L1, 0, math.pi/2)
    T12 = _create_dh_transformation(math.pi/2 + q2, 0, L2, math.pi)
    T23 = _create_dh_transformation(q3, 0, L3, math.pi)
    T34 = _create_dh_transformation(q4, 0, L4, 0)
    
    # Sequential transformation composition
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    
    # Extract end-effector position
    return T04[:3, 3]

def calculate_jacobian(joint_angles, l_values=None, a_values=None):
    """Calculate Jacobian matrix numerically using finite differences"""
    if l_values is None:
        l_values = DEFAULT_L_VALUES
    if a_values is None:
        a_values = DEFAULT_A_VALUES
        
    epsilon = 1e-6
    jacobian = np.zeros((3, 4))  # 3 DOF position, 4 DOF joints
    
    # Get current position
    current_pos = forward_kinematics_dh(joint_angles, l_values, a_values)
    
    # Calculate partial derivatives using finite differences
    for i in range(4):
        joint_angles_plus = joint_angles.copy()
        joint_angles_plus[i] += epsilon
        
        pos_plus = forward_kinematics_dh(joint_angles_plus, l_values, a_values)
        jacobian[:, i] = (pos_plus - current_pos) / epsilon
    
    return jacobian

class RobotKinematics:
    """Robot kinematics class (simplified numerical implementation)"""
    
    def __init__(self, link_lengths=None, link_offsets=None):
        self.link_lengths = link_lengths if link_lengths else DEFAULT_L_VALUES
        self.link_offsets = link_offsets if link_offsets else DEFAULT_A_VALUES
    
    def forward_position_kinematics(self, joint_angles_deg):
        """Calculate forward kinematics"""
        joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
        position = forward_kinematics_dh(joint_angles_rad, self.link_lengths, self.link_offsets)
        return position
    
    def inverse_position_kinematics(self, target_position, initial_guess_deg):
        """Solve inverse kinematics using Newton-Raphson method"""
        
        joint_angles = np.array([math.radians(angle) for angle in initial_guess_deg])
        target = np.array(target_position)
        
        for iteration in range(MAX_ITERATIONS):
            # Calculate current position
            current_position = forward_kinematics_dh(joint_angles, self.link_lengths, self.link_offsets)
            
            # Calculate error
            error = target - current_position
            
            # Check convergence
            if np.linalg.norm(error) < TOLERANCE:
                print(f"IK converged in {iteration} iterations.")
                return [math.degrees(angle) for angle in joint_angles]
            
            # Calculate Jacobian
            jacobian = calculate_jacobian(joint_angles, self.link_lengths, self.link_offsets)
            jacobian_t = jacobian.T
            
            # Damped least squares pseudo-inverse
            damping_matrix = (DAMPING_FACTOR ** 2) * np.eye(3)
            
            try:
                # Damped least squares: J^T * (J*J^T + λ²I)^(-1)
                pseudo_inverse = jacobian_t @ np.linalg.inv(jacobian @ jacobian_t + damping_matrix)
            except np.linalg.LinAlgError:
                # Fallback to regular pseudo-inverse
                pseudo_inverse = np.linalg.pinv(jacobian)
            
            # Update joint angles
            delta_q = pseudo_inverse @ error
            joint_angles += delta_q
            
            if iteration == MAX_ITERATIONS - 1:
                print("Warning: IK did not converge within maximum iterations.")
        
        return [math.degrees(angle) for angle in joint_angles]

# Initialize robot kinematics system
robot = RobotKinematics()

def check_workspace_limits(target_x, target_y, target_z):
    """Check if target is within robot's reachable workspace"""
    L1, L2, L3, L4 = DEFAULT_L_VALUES
    
    # Calculate target distance from origin in XY plane and total 3D distance
    target_distance_xy = math.sqrt(target_x**2 + target_y**2)
    target_distance_3d = math.sqrt(target_x**2 + target_y**2 + target_z**2)
    
    # Calculate workspace limits
    max_reach = L2 + L3  # Maximum horizontal reach (L1 is vertical)
    min_reach = 0.01  # Minimum reach (very close to base, avoid singularities)
    
    # Check vertical limits more accurately
    # L1 is the base height, then we can reach up or down by L2+L3
    max_height = L1 + L2 + L3  # Maximum height when fully extended up
    min_height = L1 - L2 - L3  # Minimum height when fully extended down
    
    # More sophisticated reachability check
    # The robot can reach a point if it's within the workspace sphere minus the base constraints
    horizontal_reachable = target_distance_xy <= max_reach
    vertical_reachable = min_height <= target_z <= max_height
    
    # Additional check: total distance shouldn't exceed maximum possible reach
    # accounting for the vertical offset of L1
    adjusted_target_z = target_z - L1
    effective_distance = math.sqrt(target_distance_xy**2 + adjusted_target_z**2)
    distance_reachable = effective_distance <= max_reach
    
    overall_reachable = horizontal_reachable and vertical_reachable and distance_reachable
    
    return {
        'reachable': overall_reachable,
        'target_distance_xy': target_distance_xy,
        'target_distance_3d': target_distance_3d,
        'effective_distance': effective_distance,
        'max_reach': max_reach,
        'min_reach': min_reach,
        'max_height': max_height,
        'min_height': min_height,
        'horizontal_reachable': horizontal_reachable,
        'vertical_reachable': vertical_reachable,
        'distance_reachable': distance_reachable
    }

def get_better_initial_guess(target_x, target_y, target_z):
    """Generate a better initial guess based on target position"""
    # Joint 1: Base rotation (simple atan2)
    j1_guess = math.degrees(math.atan2(target_y, target_x))
    
    # For joints 2 and 3, use a simple geometric approximation
    r_xy = math.sqrt(target_x**2 + target_y**2)
    L1, L2, L3, L4 = DEFAULT_L_VALUES
    
    # Approximate joint angles based on 2D projection
    target_distance_2d = math.sqrt(r_xy**2 + (target_z - L1)**2)
    
    if target_distance_2d > 0.001:  # Avoid division by zero
        # Use law of cosines for approximate joint 2 and 3
        try:
            # Approximate joint 2 (shoulder)
            cos_j2 = (L2**2 + target_distance_2d**2 - L3**2) / (2 * L2 * target_distance_2d)
            cos_j2 = max(-1, min(1, cos_j2))  # Clamp to valid range
            angle_to_target = math.atan2(target_z - L1, r_xy)
            j2_guess = math.degrees(angle_to_target - math.acos(cos_j2))
            
            # Approximate joint 3 (elbow)
            cos_j3 = (L2**2 + L3**2 - target_distance_2d**2) / (2 * L2 * L3)
            cos_j3 = max(-1, min(1, cos_j3))  # Clamp to valid range
            j3_guess = math.degrees(math.pi - math.acos(cos_j3))
            
        except:
            # Fallback to reasonable defaults
            j2_guess = 45.0
            j3_guess = -45.0
    else:
        j2_guess = 0.0
        j3_guess = 0.0
    
    j4_guess = 0.0  # Not used
    
    return [j1_guess, j2_guess, j3_guess, j4_guess]

def inverse_kinematics(target_x, target_y, target_z):
    """
    Calculate inverse kinematics for 4-DOF robot arm using Newton-Raphson method
    Based on DH parameters from forwardKinematics.py
    
    Returns: (j1, j2, j3, j4, j5) in radians
    Note: j4 = 0 (not used), j5 = 0 (for ROS compatibility)
    """
    
    # Check workspace limits first
    workspace_check = check_workspace_limits(target_x, target_y, target_z)
    
    if not workspace_check['reachable']:
        print(f"❌ WARNING: Target is outside reachable workspace!")
        print(f"   Target XY distance: {workspace_check['target_distance_xy']:.4f} m")
        print(f"   Effective 3D distance: {workspace_check['effective_distance']:.4f} m")
        print(f"   Max reach: {workspace_check['max_reach']:.4f} m")
        print(f"   Height limits: {workspace_check['min_height']:.4f} to {workspace_check['max_height']:.4f} m")
        print(f"   Horizontal reachable: {workspace_check['horizontal_reachable']}")
        print(f"   Vertical reachable: {workspace_check['vertical_reachable']}")
        print(f"   Distance reachable: {workspace_check['distance_reachable']}")
        
        # Scale target to be reachable
        L1 = DEFAULT_L_VALUES[0]  # Base height reference
        if not workspace_check['distance_reachable']:
            scale_factor = workspace_check['max_reach'] * 0.9 / workspace_check['effective_distance']
            target_x_scaled = target_x * scale_factor
            target_y_scaled = target_y * scale_factor
            target_z_scaled = L1 + (target_z - L1) * scale_factor  # Scale around L1 base height
        else:
            target_x_scaled = target_x
            target_y_scaled = target_y
            target_z_scaled = target_z
            
        # Clamp to height limits
        target_z_scaled = max(workspace_check['min_height'] + 0.01, 
                             min(workspace_check['max_height'] - 0.01, target_z_scaled))
        
        print(f"   Scaling target to reachable position:")
        print(f"   New target: ({target_x_scaled:.4f}, {target_y_scaled:.4f}, {target_z_scaled:.4f})")
        
        target_position = [target_x_scaled, target_y_scaled, target_z_scaled]
    else:
        print("✅ Target is within reachable workspace")
        target_position = [target_x, target_y, target_z]
    
    # Get better initial guess
    initial_guess = get_better_initial_guess(*target_position)
    print(f"Initial guess (deg): [{initial_guess[0]:.1f}, {initial_guess[1]:.1f}, {initial_guess[2]:.1f}, {initial_guess[3]:.1f}]")
    
    try:
        # Solve IK using Newton-Raphson with better initial guess
        joint_solution_deg = robot.inverse_position_kinematics(target_position, initial_guess)
        
        # Verify solution is reasonable (no extreme angles)
        for i, angle_deg in enumerate(joint_solution_deg):
            if abs(angle_deg) > 360:
                print(f"⚠️  Warning: Joint {i+1} has extreme angle: {angle_deg:.1f}°")
                # Normalize angle to reasonable range
                joint_solution_deg[i] = ((angle_deg + 180) % 360) - 180
        
        # Convert to radians
        j1, j2, j3, j4_unused = [math.radians(angle) for angle in joint_solution_deg]
        
        # j4 is not used (as confirmed in milestone_2.py), j5 = 0
        j4 = 0.0
        j5 = 0.0
        
        return j1, j2, j3, j4, j5
        
    except Exception as e:
        print(f"IK solver error: {e}")
        # Return safe default values
        return 0.0, 0.0, 0.0, 0.0, 0.0

def forward_kinematics_check(j1, j2, j3, j4, j5):
    """
    Forward kinematics check using the correct DH parameters
    """
    # Convert to degrees for robot.forward_position_kinematics
    joint_angles_deg = [math.degrees(j1), math.degrees(j2), math.degrees(j3), math.degrees(j4)]
    
    try:
        position = robot.forward_position_kinematics(joint_angles_deg)
        return position[0], position[1], position[2]
    except Exception as e:
        print(f"Forward kinematics check error: {e}")
        return 0.0, 0.0, 0.0

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
    # normalize into local floats so static checkers know they're numeric
    tx = float(target_x)
    ty = float(target_y) # type: ignore
    tz = float(target_z) # type: ignore

    # Calculate inverse kinematics
    try:
        j1_val, j2_val, j3_val, j4_val, j5_val = inverse_kinematics(tx, ty, tz)
        
        print(f"\nCalculated joint angles:")
        print(f"  Joint 1 = {math.degrees(j1_val):+7.2f}° ({j1_val:+.4f} rad)")
        print(f"  Joint 2 = {math.degrees(j2_val):+7.2f}° ({j2_val:+.4f} rad)")
        print(f"  Joint 3 = {math.degrees(j3_val):+7.2f}° ({j3_val:+.4f} rad)")
        print(f"  Joint 4 = {math.degrees(j4_val):+7.2f}° ({j4_val:+.4f} rad)")
        print(f"  Joint 5 = {math.degrees(j5_val):+7.2f}° ({j5_val:+.4f} rad)")
        
        # Verify with forward kinematics
        fk_x, fk_y, fk_z = forward_kinematics_check(j1_val, j2_val, j3_val, j4_val, j5_val)
        # compute error against normalized target coordinates
        error = math.sqrt((fk_x - tx)**2 + (fk_y - ty)**2 + (fk_z - tz)**2)

        print(f"\nForward kinematics verification:")
        print(f"  Calculated: x={fk_x:.4f}, y={fk_y:.4f}, z={fk_z:.4f}")
        print(f"  Target:     x={target_x:.4f}, y={target_y:.4f}, z={target_z:.4f}")
        print(f"  Error:      {error:.4f} m")

        # If we have current joint info, check whether robot is already at the target
        position_tolerance = 0.005  # meters
        joint_tolerance = 0.02  # radians (~1 deg)
        if current_joints is not None:
            try:
                if len(current_joints) >= 3 and current_joints[0] is not None:
                    # use available joints (fill missing with zeros)
                    curr = [ (current_joints[i] if i < len(current_joints) and current_joints[i] is not None else 0.0) for i in range(5) ]
                    curr_fk_x, curr_fk_y, curr_fk_z = forward_kinematics_check(curr[0], curr[1], curr[2], curr[3], curr[4])
                    pos_err_now = math.sqrt((curr_fk_x - tx)**2 + (curr_fk_y - ty)**2 + (curr_fk_z - tz)**2)
                    if pos_err_now <= position_tolerance:
                        print("Robot already at target (within tolerance). No command will be sent.")
                        print("-" * 60)
                        continue
                    # also compare joint angles: if already close, skip
                    joint_diffs = [abs(curr[i] - v) for i, v in enumerate([j1_val, j2_val, j3_val, j4_val, j5_val])]
                    if all(d <= joint_tolerance for d in joint_diffs):
                        print("Joint angles already within tolerance. No command will be sent.")
                        print("-" * 60)
                        continue
            except Exception:
                # if any failure, proceed to ask user
                pass

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