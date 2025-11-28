#!/usr/bin/env python3
"""
Cubic Polynomial Trajectory Planning Node

This node performs trajectory planning using cubic polynomials for joint-space motion.
It uses the inverse kinematics node to calculate initial and final joint angles,
then generates smooth trajectories between these configurations.

Based on milestone_2.py structure with 4-DOF robot (Joint_4 not used).

Author: Team 13
Date: November 28, 2025
"""

import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import sys
import os

# Import inverse kinematics functions
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Import only the calculation functions, not the interactive node
def import_ik_functions():
    """Import IK functions without initializing ROS node or running interactive mode"""
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location("ik_module", 
            os.path.join(os.path.dirname(__file__), "inverseKinematics.py"))
        ik_module = importlib.util.module_from_spec(spec) # type: ignore
        
        # Temporarily disable node initialization by monkey-patching rospy.init_node
        import rospy as rospy_temp
        original_init_node = rospy_temp.init_node
        rospy_temp.init_node = lambda *args, **kwargs: None
        
        spec.loader.exec_module(ik_module) # type: ignore
        
        # Restore original function
        rospy_temp.init_node = original_init_node
        
        return ik_module.inverse_kinematics, ik_module.forward_kinematics_check
    except Exception as e:
        print(f"❌ Error importing inverse kinematics: {e}")
        print("Make sure inverseKinematics.py is in the same directory.")
        sys.exit(1)

inverse_kinematics, forward_kinematics_check = import_ik_functions()
print("✅ Successfully imported inverse kinematics functions")

class CubicPolynomialTrajectory:
    """
    Cubic polynomial trajectory generator for joint-space planning
    
    For each joint, generates a cubic polynomial:
    q(t) = a0 + a1*t + a2*t² + a3*t³
    
    With boundary conditions:
    - Initial position: q(0) = q0
    - Final position: q(T) = qf  
    - Initial velocity: q'(0) = v0 (usually 0)
    - Final velocity: q'(T) = vf (usually 0)
    """
    
    def __init__(self):
        self.coefficients = None
        self.duration = 10.0
        self.num_joints = 5  # 5 joints but only 4 DOF (Joint_4 not used)
        
    def calculate_coefficients(self, q0, qf, duration, v0=None, vf=None):
        """
        Calculate cubic polynomial coefficients for each joint
        
        Args:
            q0: Initial joint positions [rad] (list of 5 values)
            qf: Final joint positions [rad] (list of 5 values)
            duration: Trajectory duration [s]
            v0: Initial joint velocities [rad/s] (default: all zeros)
            vf: Final joint velocities [rad/s] (default: all zeros)
            
        Returns:
            coefficients: [5 x 4] array of [a0, a1, a2, a3] for each joint
        """
        
        # Default to zero velocities
        if v0 is None:
            v0 = [0.0] * self.num_joints
        if vf is None:
            vf = [0.0] * self.num_joints
            
        self.duration = duration
        T = duration
        
        # Calculate coefficients for each joint
        coefficients = np.zeros((self.num_joints, 4))
        
        for i in range(self.num_joints):
            # Boundary conditions:
            # q(0) = q0[i]     =>  a0 = q0[i]
            # q'(0) = v0[i]    =>  a1 = v0[i] 
            # q(T) = qf[i]     =>  a0 + a1*T + a2*T² + a3*T³ = qf[i]
            # q'(T) = vf[i]    =>  a1 + 2*a2*T + 3*a3*T² = vf[i]
            
            a0 = q0[i]
            a1 = v0[i]
            
            # Solve for a2 and a3 using the final conditions
            # Matrix form: [T² T³] [a2] = [qf - a0 - a1*T]
            #              [2T 3T²] [a3]   [vf - a1]
            
            A = np.array([[T**2, T**3],
                         [2*T, 3*T**2]])
            b = np.array([qf[i] - a0 - a1*T,
                         vf[i] - a1])
            
            try:
                a2_a3 = np.linalg.solve(A, b)
                a2, a3 = a2_a3
            except np.linalg.LinAlgError:
                print(f"⚠️ Numerical issue for joint {i+1}, using linear interpolation")
                a2 = 0.0
                a3 = 0.0
                a1 = (qf[i] - a0) / T
            
            coefficients[i] = [a0, a1, a2, a3]
            
        self.coefficients = coefficients
        return coefficients
    
    def evaluate_trajectory(self, t):
        """
        Evaluate trajectory at time t
        
        Args:
            t: Time [s]
            
        Returns:
            positions: Joint positions [rad] (list of 5 values)
            velocities: Joint velocities [rad/s] (list of 5 values)
            accelerations: Joint accelerations [rad/s²] (list of 5 values)
        """
        
        if self.coefficients is None:
            raise ValueError("Trajectory coefficients not calculated. Call calculate_coefficients first.")
            
        # Clamp time to trajectory duration
        t = max(0.0, min(t, self.duration))
        
        positions = []
        velocities = []
        accelerations = []
        
        for i in range(self.num_joints):
            a0, a1, a2, a3 = self.coefficients[i]
            
            # Position: q(t) = a0 + a1*t + a2*t² + a3*t³
            q = a0 + a1*t + a2*t**2 + a3*t**3
            
            # Velocity: q'(t) = a1 + 2*a2*t + 3*a3*t²
            v = a1 + 2*a2*t + 3*a3*t**2
            
            # Acceleration: q''(t) = 2*a2 + 6*a3*t
            a = 2*a2 + 6*a3*t
            
            positions.append(q)
            velocities.append(v)
            accelerations.append(a)
            
        return positions, velocities, accelerations


class CubicTrajectoryPlannerNode:
    """
    ROS node for cubic polynomial trajectory planning
    Uses the same publisher structure as milestone_2.py
    """
    
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('cubic_trajectory_planner', anonymous=False)
        
        # Publishers for joint commands (same as milestone_2.py)
        self.pub1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
        self.pub2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
        self.pub3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
        self.pub4 = rospy.Publisher('/Joint_4/command', Float64, queue_size=10)
        self.pub5 = rospy.Publisher('/Joint_5/command', Float64, queue_size=10)
        
        # Publisher for trajectory status
        self.pose_pub = rospy.Publisher('/trajectory_current_pose', PoseStamped, queue_size=10)
        
        # Trajectory generator
        self.trajectory = CubicPolynomialTrajectory()
        
        # State variables
        self.executing = False
        
        # Parameters
        self.control_frequency = 10.0  # Hz (slower for better visualization)
        
        # Wait for publishers to connect
        rospy.sleep(1.0)
        
        rospy.loginfo("✅ Cubic Trajectory Planner Node initialized")
        rospy.loginfo("Publishers connected - ready to move robot!")
        
        self.print_usage_instructions()
    
    def print_usage_instructions(self):
        """Print usage instructions"""
        print("=" * 70)
        print("         CUBIC POLYNOMIAL TRAJECTORY PLANNER")
        print("=" * 70)
        print("This node creates smooth trajectories between two 3D positions")
        print("using cubic polynomials for joint-space motion planning.")
        print()
        print("🤖 Robot Configuration: 4-DOF (Joint_4 not used)")
        print("📐 Uses inverse kinematics for position-to-joint conversion")
        print("📈 Generates cubic polynomial trajectories for smooth motion")
        print("=" * 70)
    
    def get_user_input(self):
        """Get start and end positions from user"""
        print("\n�� TRAJECTORY PLANNING INPUT")
        print("=" * 40)
        
        try:
            print("Enter START position (x, y, z in meters):")
            start_x = float(input("  Start X: "))
            start_y = float(input("  Start Y: "))
            start_z = float(input("  Start Z: "))
            start_pos = (start_x, start_y, start_z)
            
            print("\nEnter END position (x, y, z in meters):")
            end_x = float(input("  End X: "))
            end_y = float(input("  End Y: "))
            end_z = float(input("  End Z: "))
            end_pos = (end_x, end_y, end_z)
            
            print("\nTrajectory parameters:")
            duration_input = input("  Duration in seconds (default: 10.0): ").strip()
            duration = float(duration_input) if duration_input else 10.0
            
            return start_pos, end_pos, duration
            
        except ValueError:
            print("❌ Invalid input. Please enter numeric values.")
            return None, None, None
        except KeyboardInterrupt:
            print("\n👋 Cancelled by user.")
            return None, None, None
    
    def calculate_trajectory_joints(self, start_pos, end_pos):
        """
        Calculate start and end joint angles using inverse kinematics
        
        Args:
            start_pos: (x, y, z) start position
            end_pos: (x, y, z) end position
            
        Returns:
            start_joints: Initial joint angles [rad] (list of 5)
            end_joints: Final joint angles [rad] (list of 5)
        """
        
        print(f"\n📐 INVERSE KINEMATICS CALCULATION")
        print(f"Start position: ({start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f})")
        print(f"End position:   ({end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f})")
        
        try:
            # Calculate start joint angles
            start_joints_ik = inverse_kinematics(*start_pos)
            start_joints = list(start_joints_ik)
            
            # Calculate end joint angles  
            end_joints_ik = inverse_kinematics(*end_pos)
            end_joints = list(end_joints_ik)
            
            # Ensure we have 5 joints (pad with zeros if needed)
            while len(start_joints) < 5:
                start_joints.append(0.0)
            while len(end_joints) < 5:
                end_joints.append(0.0)
            
            # Joint_4 is not used (set to 0 as in milestone_2.py)
            start_joints[3] = 0.0
            end_joints[3] = 0.0
            
            # Log joint angles
            print("✅ Inverse kinematics successful!")
            print("Start joint angles (deg):")
            for i, angle in enumerate(start_joints):
                used = "" if i != 3 else " (not used)"
                print(f"  Joint_{i+1} = {math.degrees(angle):+7.2f}°{used}")
            
            print("End joint angles (deg):")
            for i, angle in enumerate(end_joints):
                used = "" if i != 3 else " (not used)"
                print(f"  Joint_{i+1} = {math.degrees(angle):+7.2f}°{used}")
            
            return start_joints, end_joints
            
        except Exception as e:
            print(f"❌ Inverse kinematics failed: {e}")
            raise
    
    def publish_joint_commands(self, joint_positions):
        """
        Publish joint commands using the same structure as milestone_2.py
        
        Args:
            joint_positions: List of 5 joint angles [rad]
        """
        
        # Create Float64 messages (same as milestone_2.py)
        joint_1 = Float64(joint_positions[0])
        joint_2 = Float64(joint_positions[1]) 
        joint_3 = Float64(joint_positions[2])
        joint_4 = Float64(joint_positions[3])  # This servo will not be used
        joint_5 = Float64(joint_positions[4])
        
        # Publish commands
        self.pub1.publish(joint_1)
        self.pub2.publish(joint_2)
        self.pub3.publish(joint_3)
        self.pub4.publish(joint_4)
        self.pub5.publish(joint_5)
    
    def execute_trajectory(self, start_pos, end_pos, duration):
        """
        Execute cubic polynomial trajectory
        
        Args:
            start_pos: (x, y, z) start position
            end_pos: (x, y, z) end position  
            duration: Trajectory duration [s]
        """
        
        try:
            # Calculate joint trajectories
            start_joints, end_joints = self.calculate_trajectory_joints(start_pos, end_pos)
            
            # Calculate trajectory coefficients
            print(f"\n📈 TRAJECTORY GENERATION")
            print(f"Generating cubic polynomial trajectory...")
            print(f"Duration: {duration} seconds")
            print(f"Control frequency: {self.control_frequency} Hz")
            
            self.trajectory.calculate_coefficients(start_joints, end_joints, duration)
            
            # Execute trajectory
            print(f"\n🎯 TRAJECTORY EXECUTION")
            print("Starting smooth motion...")
            
            self.executing = True
            rate = rospy.Rate(self.control_frequency)
            start_time = rospy.Time.now()
            
            step_count = 0
            
            while not rospy.is_shutdown() and self.executing:
                # Calculate elapsed time
                elapsed = (rospy.Time.now() - start_time).to_sec()
                
                # Check if trajectory is complete
                if elapsed >= duration:
                    print("\n✅ Trajectory completed successfully!")
                    break
                
                # Evaluate trajectory at current time
                positions, velocities, accelerations = self.trajectory.evaluate_trajectory(elapsed)
                
                # Publish joint commands
                self.publish_joint_commands(positions)
                
                # Print progress periodically
                if step_count % int(self.control_frequency / 2) == 0:  # Every 0.5 seconds
                    progress = (elapsed / duration) * 100
                    print(f"Progress: {progress:.1f}% - Time: {elapsed:.1f}s/{duration:.1f}s")
                    
                    # Show current joint angles
                    joint_degrees = [math.degrees(pos) for pos in positions]
                    print(f"Joints (deg): [{joint_degrees[0]:+6.1f}, {joint_degrees[1]:+6.1f}, {joint_degrees[2]:+6.1f}, {joint_degrees[3]:+6.1f}, {joint_degrees[4]:+6.1f}]")
                    
                    # Show current end-effector position if possible
                    try:
                        curr_x, curr_y, curr_z = forward_kinematics_check(*positions)
                        print(f"Current position: ({curr_x:.3f}, {curr_y:.3f}, {curr_z:.3f})")
                    except:
                        pass
                
                step_count += 1
                rate.sleep()
                
            self.executing = False
            print("🎉 Trajectory execution finished!")
            return True
            
        except Exception as e:
            print(f"❌ Trajectory execution failed: {e}")
            self.executing = False
            return False
    
    def run_interactive_mode(self):
        """Run interactive trajectory planning"""
        
        while not rospy.is_shutdown():
            try:
                # Get user input
                start_pos, end_pos, duration = self.get_user_input()
                
                if start_pos is None:  # User cancelled
                    break
                
                # Show summary
                print(f"\n📊 TRAJECTORY SUMMARY")
                print(f"Start position: ({start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f})")
                print(f"End position:   ({end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f})") # type: ignore
                print(f"Duration: {duration} seconds")
                
                # Confirm execution
                proceed = input("\nProceed with trajectory execution? (y/n): ").strip().lower()
                if proceed not in ['y', 'yes']:
                    print("Trajectory cancelled.")
                    continue
                
                # Execute trajectory
                success = self.execute_trajectory(start_pos, end_pos, duration)
                
                if success:
                    print("\n✅ Trajectory completed successfully!")
                else:
                    print("\n❌ Trajectory execution failed!")
                
                # Ask if user wants to run another trajectory
                again = input("\nRun another trajectory? (y/n): ").strip().lower()
                if again not in ['y', 'yes']:
                    break
                    
            except KeyboardInterrupt:
                print("\n👋 Interrupted by user.")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
                import traceback
                traceback.print_exc()
        
        print("\n👋 Cubic Trajectory Planner finished.")


def run_example_trajectory(planner):
    """Run a predefined example trajectory"""
    print("\n🎬 RUNNING EXAMPLE TRAJECTORY")
    print("=" * 40)
    
    # Safe example positions
    start_pos = (0.15, 0.0, 0.25)
    end_pos = (0.25, 0.0, 0.25)
    duration = 10.0
    
    print(f"Example: Simple forward motion")
    print(f"Start: {start_pos}")
    print(f"End: {end_pos}")
    print(f"Duration: {duration}s")
    
    success = planner.execute_trajectory(start_pos, end_pos, duration)
    
    if success:
        print("\n✅ Example trajectory completed successfully!")
    else:
        print("\n❌ Example trajectory failed!")
    
    return success


def main():
    """Main function"""
    try:
        print("=" * 70)
        print("       CUBIC POLYNOMIAL TRAJECTORY PLANNER")
        print("=" * 70)
        print("This node creates smooth joint-space trajectories using cubic polynomials.")
        print()
        print("⚠️  REQUIREMENTS:")
        print("1. Gazebo simulation must be running")
        print("2. Robot controllers must be loaded")
        print("3. inverseKinematics.py must be available")
        print()
        
        # Create trajectory planner node
        planner = CubicTrajectoryPlannerNode()
        
        # Main menu
        print("CHOOSE OPERATION MODE:")
        print("1. Interactive mode (enter custom positions)")
        print("2. Run example trajectory")
        print("3. Exit")
        
        choice = input("\nEnter choice (1-3): ").strip()
        
        if choice == "1":
            planner.run_interactive_mode()
            
        elif choice == "2":
            run_example_trajectory(planner)
            
        elif choice == "3":
            print("👋 Goodbye!")
            
        else:
            print("❌ Invalid choice. Please enter 1-3.")
        
    except KeyboardInterrupt:
        print("\n👋 Interrupted by user. Goodbye!")
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
