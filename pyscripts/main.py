"""
Main script to demonstrate forward kinematics calculation for a 5-DOF robot arm.
This script uses DH parameters to calculate the end effector position.
"""

import numpy as np
import math
from sympy import pi
from kinematics import get_dh_parameters, create_joint_matrices, forward_kinematics, calculate_forward_kinematics, get_symbolic_forward_kinematics


def calculate_end_effector_position(q1, q2, q3, q4, q5, equations=None):
    """
    Calculate end effector position for given joint angles.
    
    Parameters:
    -----------
    q1, q2, q3, q4, q5 : float
        Joint angles in radians
    equations : tuple, optional
        Symbolic equations (not used in this implementation)
    
    Returns:
    --------
    tuple
        (x, y, z) position of end effector
    """
    
    # Use the calculate_forward_kinematics function
    joint_angles = [q1, q2, q3, q4, q5]
    return calculate_forward_kinematics(joint_angles)


def get_forward_kinematics_equations():
    """
    Get symbolic forward kinematics equations.
    
    Returns:
    --------
    tuple
        Symbolic equations for x, y, z position
    """
    
    # Use the get_symbolic_forward_kinematics function
    transformation_matrix, equations = get_symbolic_forward_kinematics()
    return equations


def demonstrate_forward_kinematics():
    """
    Demonstrate the complete forward kinematics calculation process.
    """
    
    print("=" * 60)
    print("4-DOF Robot Arm Forward Kinematics Calculator")
    print("=" * 60)
    
    # Show DH parameters
    print("\n1. DH Parameters:")
    print("Joint |  θ    |    d     |    a    |   α   ")
    print("------|-------|----------|---------|-------")
    print("  1   |  q1   |  0.0     |   0.0   |  0.0  ")
    print("  2   |  q2   | 0.04975  |   0.0   | π/2   ")
    print("  3   |  q3   |  0.0     |  0.140  |  0.0  ")
    print("  4   |  q4   |  0.0     |   0.0   | π/2   ")
    print("  5   |  q5   | 0.13289  |   0.0   |  0.0  ")
    
    # Get DH parameters
    dh_params = get_dh_parameters()
    
    print("\n2. Creating individual joint transformation matrices...")
    joint_matrices = create_joint_matrices(dh_params)
    
    print(f"\n3. Calculating forward kinematics (T_total = T1 * T2 * T3 * T4 * T5)...")
    total_T, end_effector_equations = forward_kinematics(joint_matrices)
    
    print(f"\n4. Forward Kinematics Equations:")
    print(f"   x(q1,q2,q3,q4,q5) = {end_effector_equations[0]}")
    print(f"   y(q1,q2,q3,q4,q5) = {end_effector_equations[1]}")
    print(f"   z(q1,q2,q3,q4,q5) = {end_effector_equations[2]}")
    
    return end_effector_equations


def test_specific_configurations():
    """
    Test the forward kinematics with specific joint configurations.
    """
    
    print("\n" + "=" * 60)
    print("Testing Specific Joint Configurations")
    print("=" * 60)
    
    # Get equations once for efficiency
    equations = get_forward_kinematics_equations()
    
    # Define test configurations
    test_configs = [
        {
            'name': 'Home Position (All zeros)',
            'joints': (0, 0, 0, 0, 0),
            'description': 'All joints at 0 radians'
        },
        {
            'name': 'Joint 1 at 90°',
            'joints': (pi/2, 0, 0, 0, 0),
            'description': 'First joint rotated 90 degrees'
        },
        {
            'name': 'Joint 2 at 90°',
            'joints': (0, pi/2, 0, 0, 0),
            'description': 'Second joint rotated 90 degrees'
        },
        {
            'name': 'Joints 2&3 at 45°',
            'joints': (0, pi/4, pi/4, 0, 0),
            'description': 'Second and third joints at 45 degrees'
        },
        {
            'name': 'All joints at 30°',
            'joints': (pi/6, pi/6, pi/6, pi/6, pi/6),
            'description': 'All joints at 30 degrees'
        }
    ]
    
    for i, config in enumerate(test_configs, 1):
        print(f"\nTest {i}: {config['name']}")
        print(f"Description: {config['description']}")
        
        q1, q2, q3, q4, q5 = config['joints']
        print(f"Joint angles: q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}, q4={q4:.4f}, q5={q5:.4f}")
        print(f"              ({q1*180/pi:.1f}°, {q2*180/pi:.1f}°, {q3*180/pi:.1f}°, {q4*180/pi:.1f}°, {q5*180/pi:.1f}°)")
        
        # Calculate end effector position
        x, y, z = calculate_end_effector_position(q1, q2, q3, q4, q5, equations)
        
        print(f"End effector position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        # Calculate reach (distance from base)
        reach = np.sqrt(x**2 + y**2 + z**2)
        print(f"Reach from origin: {reach:.3f}")


def interactive_mode():
    """
    Interactive mode to calculate end effector position for user-specified joint angles.
    """
    
    print("\n" + "=" * 60)
    print("Interactive Mode - Joint Angle Input")
    print("=" * 60)
    
    # Ask user for input format preference
    print("Choose input format:")
    print("1. Radians")
    print("2. Degrees")
    
    while True:
        try:
            format_choice = input("Enter choice (1 or 2): ").strip()
            if format_choice in ['1', '2']:
                break
            else:
                print("Please enter 1 or 2")
        except KeyboardInterrupt:
            print("\nExiting interactive mode.")
            return
    
    use_degrees = (format_choice == '2')
    unit = "degrees" if use_degrees else "radians"
    unit_short = "deg" if use_degrees else "rad"
    
    print(f"\nEnter joint angles in {unit}")
    print("Commands: 'q' or 'quit' to exit, 'help' for examples")
    
    # Pre-calculate equations for efficiency
    equations = get_forward_kinematics_equations()
    
    calculation_count = 0
    
    while True:
        try:
            print(f"\n--- Calculation #{calculation_count + 1} ---")
            
            # Get joint angles with improved input handling
            angles = []
            joint_names = ['q1', 'q2', 'q3', 'q4', 'q5']
            
            for joint_name in joint_names:
                while True:
                    try:
                        user_input = input(f"{joint_name} ({unit_short}): ").strip().lower()
                        
                        if user_input in ['q', 'quit']:
                            print("\nExiting interactive mode.")
                            return
                        elif user_input == 'help':
                            print("\nExample inputs:")
                            if use_degrees:
                                print("  0, 90, -45, 180, 0 (for degrees)")
                                print("  Common angles: 0°, 30°, 45°, 90°, 180°")
                            else:
                                print("  0, 1.57, -0.785, 3.14, 0 (for radians)")
                                print("  Common angles: 0, π/6≈0.52, π/4≈0.785, π/2≈1.57, π≈3.14")
                            continue
                        
                        angle_value = float(user_input)
                        angles.append(angle_value)
                        break
                        
                    except ValueError:
                        print(f"Please enter a valid number for {joint_name}")
            
            # Convert degrees to radians if needed
            if use_degrees:
                angles_rad = [angle * pi / 180 for angle in angles]
            else:
                angles_rad = angles
            
            q1, q2, q3, q4, q5 = angles_rad
            
            # Calculate end effector position
            x, y, z = calculate_end_effector_position(q1, q2, q3, q4, q5, equations)
            
            # Display results
            print(f"\n{'='*40}")
            print("RESULTS")
            print(f"{'='*40}")
            
            if use_degrees:
                print(f"Input angles (deg): q1={angles[0]:.2f}°, q2={angles[1]:.2f}°, q3={angles[2]:.2f}°, q4={angles[3]:.2f}°, q5={angles[4]:.2f}°")
                print(f"Angles in radians:  q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}, q4={q4:.4f}, q5={q5:.4f}")
            else:
                print(f"Input angles (rad): q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}, q4={q4:.4f}, q5={q5:.4f}")
                print(f"Angles in degrees:  q1={q1*180/pi:.1f}°, q2={q2*180/pi:.1f}°, q3={q3*180/pi:.1f}°, q4={q4*180/pi:.1f}°, q5={q5*180/pi:.1f}°")
            
            print(f"\nEnd effector position:")
            print(f"  x = {x:.3f}")
            print(f"  y = {y:.3f}")
            print(f"  z = {z:.3f}")
            
            # Calculate reach and other useful metrics
            reach = np.sqrt(x**2 + y**2 + z**2)
            xy_distance = np.sqrt(x**2 + y**2)
            
            print(f"\nAdditional info:")
            print(f"  3D distance from origin: {reach:.3f}")
            print(f"  XY plane distance: {xy_distance:.3f}")
            print(f"  Height (Z): {z:.3f}")
            
            if xy_distance > 0:
                angle_from_x = np.arctan2(y, x) * 180 / pi
                print(f"  Angle from X-axis in XY plane: {angle_from_x:.1f}°")
            
            calculation_count += 1
            
            # Ask if user wants to continue
            print(f"\n{'='*40}")
            continue_choice = input("Calculate another position? (y/n): ").strip().lower()
            if continue_choice in ['n', 'no', 'quit', 'q']:
                break
                
        except ValueError:
            print("Please enter valid numbers for joint angles.")
        except KeyboardInterrupt:
            print("\nExiting interactive mode.")
            break
        except Exception as e:
            print(f"Error in calculation: {e}")
    
    print(f"\nCompleted {calculation_count} calculations. Goodbye!")


def main():
    """
    Main function to run the forward kinematics demonstration.
    """
    
    try:
        print("=" * 60)
        print("4-DOF Robot Arm Forward Kinematics Calculator")
        print("=" * 60)
        
        # Ask user what they want to do
        print("\nChoose an option:")
        print("1. Run complete demonstration (show equations + test configurations)")
        print("2. Interactive mode (input your own joint angles)")
        print("3. Both demonstration and interactive mode")
        
        while True:
            try:
                choice = input("\nEnter your choice (1, 2, or 3): ").strip()
                if choice in ['1', '2', '3']:
                    break
                else:
                    print("Please enter 1, 2, or 3")
            except KeyboardInterrupt:
                print("\nExiting...")
                return
        
        if choice in ['1', '3']:
            # Run demonstration
            equations = demonstrate_forward_kinematics()
            test_specific_configurations()
            
            print("\n" + "=" * 60)
            print("Demonstration Complete!")
            print("=" * 60)
        
        if choice in ['2', '3']:
            # Run interactive mode
            interactive_mode()
        
        print("\nThank you for using the Forward Kinematics Calculator!")
        
    except Exception as e:
        print(f"Error during calculation: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
