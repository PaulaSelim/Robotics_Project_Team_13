"""Forward Kinematics Calculator for 4-DOF Revolute Manipulator.

This module provides symbolic and numerical calculations for position, velocity,
and acceleration kinematics of a 4-DOF revolute manipulator using Denavit-Hartenberg
parameters and geometric Jacobian methods.
"""

import sys
from typing import Tuple, List, Callable, Any, Optional

import numpy as np
import sympy as sp  # type: ignore

# Constants
DOF = 4
MAX_ITERATIONS = 200
TOLERANCE = 1e-6
DAMPING_FACTOR = 0.01

# Robot parameters
DEFAULT_L_VALUES = [0.04355, 0.140, 0.1329, 0.0]  # [L1, L2, L3, L4]
DEFAULT_A_VALUES = [0.0, -0.03786, -0.0387, 0.0]  # [a1, a2, a3, a4]


def _create_dh_transformation(theta: sp.Expr, d: sp.Expr, a: sp.Expr, alpha: sp.Expr) -> sp.Matrix:
    """Create Denavit-Hartenberg transformation matrix.
    
    Args:
        theta: Joint angle
        d: Link offset
        a: Link length
        alpha: Link twist angle
        
    Returns:
        4x4 transformation matrix
    """
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],  # type: ignore
        [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],  # type: ignore
        [0, sp.sin(alpha), sp.cos(alpha), d],
        [0, 0, 0, 1]
    ])


def _calculate_jacobian_column(z_axis: sp.Matrix, position_diff: sp.Matrix) -> sp.Matrix:
    """Calculate a column of the geometric Jacobian for revolute joint.
    
    Args:
        z_axis: Z-axis of the joint frame
        position_diff: Vector from joint origin to end-effector
        
    Returns:
        3x1 Jacobian column vector
    """
    return z_axis.cross(position_diff)


def symbolic_kinematics_calculation() -> Tuple[Any, Any, Callable, Callable, Callable, List, List]:
    """Perform symbolic calculations for position, velocity, and acceleration kinematics.
    
    Returns:
        Tuple containing:
        - X_symbolic: Symbolic position expression
        - J_symbolic: Symbolic Jacobian expression
        - X_func: Numerical position function
        - J_func: Numerical Jacobian function
        - X_acc_bias_func: Numerical acceleration bias function
        - all_pos_symbols: Position symbols list
        - all_vel_symbols: Velocity symbols list
    """
    print("Computing comprehensive symbolic kinematics analysis...")
    print("Deriving Position, Jacobian, and Acceleration expressions - This may take some time...")

    # Initialize temporal and symbolic variables
    temporal_parameter = sp.symbols('t')
    
    # Create time-dependent joint angle functions
    joint_angle_functions = []
    for joint_index in range(1, DOF + 1):
        joint_angle_functions.append(sp.Function(f'q{joint_index}')(temporal_parameter))
    q1_t, q2_t, q3_t, q4_t = joint_angle_functions
    
    # Derive velocity functions via differentiation
    joint_velocity_functions = []
    for angle_func in joint_angle_functions:
        joint_velocity_functions.append(sp.diff(angle_func, temporal_parameter))
    q1_dot_t, q2_dot_t, q3_dot_t, q4_dot_t = joint_velocity_functions
    
    # Derive acceleration functions via second-order differentiation
    joint_acceleration_functions = []
    for velocity_func in joint_velocity_functions:
        joint_acceleration_functions.append(sp.diff(velocity_func, temporal_parameter))
    q1_ddot_t, q2_ddot_t, q3_ddot_t, q4_ddot_t = joint_acceleration_functions
    
    # Standalone symbols for numerical functions
    joint_symbols = sp.symbols('q1:5')
    q1, q2, q3, q4 = joint_symbols
    
    velocity_symbols = sp.symbols('q1_dot q2_dot q3_dot q4_dot')
    q1_dot, q2_dot, q3_dot, q4_dot = velocity_symbols
    
    acceleration_symbols = sp.symbols('q1_ddot q2_ddot q3_ddot q4_ddot')
    q1_ddot, q2_ddot, q3_ddot, q4_ddot = acceleration_symbols
    
    # Link parameters
    link_length_symbols = sp.symbols('L1:5')
    L1, L2, L3, L4 = link_length_symbols
    
    link_offset_symbols = sp.symbols('a1:5')
    a1, a2, a3, a4 = link_offset_symbols
    
    # Symbol mapping for substitution
    symbol_mappings = {
        **dict(zip(joint_angle_functions, joint_symbols)),
        **dict(zip(joint_velocity_functions, velocity_symbols)),
        **dict(zip(joint_acceleration_functions, acceleration_symbols))
    }

    print("Phase 1/3: Forward Position Kinematics Computation...")
    
    # Denavit-Hartenberg transformation matrices (preserving your DH table)
    # Frame 0 to 1: (q1, L1, 0, pi/2)
    # Frame 1 to 2: (pi/2 + q2, 0, L2, pi)
    # Frame 2 to 3: (q3, 0, L3, pi)
    # Frame 3 to 4: (q4, 0, L4, 0)
    
    transformation_01 = _create_dh_transformation(q1_t, L1, 0, sp.pi/2)  # type: ignore
    transformation_12 = _create_dh_transformation(sp.pi/2 + q2_t, 0, L2, sp.pi)  # type: ignore
    transformation_23 = _create_dh_transformation(q3_t, 0, L3, sp.pi)  # type: ignore
    transformation_34 = _create_dh_transformation(q4_t, 0, L4, 0)  # type: ignore

    # Sequential transformation composition
    transformation_02 = sp.simplify(transformation_01 * transformation_12)
    transformation_03 = sp.simplify(transformation_02 * transformation_23)
    transformation_04 = sp.simplify(transformation_03 * transformation_34)

    # Extract end-effector position vector
    end_effector_position_t = transformation_04[:3, 3]

    print("Phase 2/3: Geometric Jacobian Matrix Derivation...")
    
    # Frame coordinate origins and rotation axes for Jacobian computation
    coordinate_frame_origins = [
        sp.Matrix([0, 0, 0]),         # Base frame origin O0
        transformation_01[:3, 3],     # Frame 1 origin O1
        transformation_02[:3, 3],     # Frame 2 origin O2
        transformation_03[:3, 3],     # Frame 3 origin O3
        transformation_04[:3, 3]      # End-effector origin ON
    ]
    
    rotation_axes_z = [
        sp.Matrix([0, 0, 1]),         # Base z-axis Z0
        transformation_01[:3, 2],     # Frame 1 z-axis Z1
        transformation_02[:3, 2],     # Frame 2 z-axis Z2
        transformation_03[:3, 2]      # Frame 3 z-axis Z3
    ]
    
    # Calculate Jacobian columns (all revolute joints)
    jacobian_column_vectors = []
    for joint_index in range(DOF):
        position_vector_difference = coordinate_frame_origins[DOF] - coordinate_frame_origins[joint_index]
        jacobian_column = _calculate_jacobian_column(rotation_axes_z[joint_index], position_vector_difference)
        jacobian_column_vectors.append(jacobian_column)
    
    # Assemble complete Jacobian matrix
    jacobian_symbolic_t = sp.Matrix.hstack(*jacobian_column_vectors)

    print("Phase 3/3: Acceleration Bias Term Computation (J_dot * q_dot)...")
    
    # Calculate acceleration bias term
    joint_velocity_vector = sp.Matrix(joint_velocity_functions)
    joint_acceleration_vector = sp.Matrix(joint_acceleration_functions)
    
    # End-effector velocity: X_dot = J * q_dot
    end_effector_velocity_t = jacobian_symbolic_t * joint_velocity_vector
    
    # End-effector acceleration: X_ddot = J_dot * q_dot + J * q_ddot
    end_effector_acceleration_t = sp.diff(end_effector_velocity_t, temporal_parameter)
    
    # Isolate bias term: J_dot * q_dot = X_ddot - J * q_ddot
    jacobian_q_ddot_term = jacobian_symbolic_t * joint_acceleration_vector
    acceleration_bias_t = sp.simplify(end_effector_acceleration_t - jacobian_q_ddot_term)

    print("Performing symbol substitution for numerical function generation...")
    
    # Replace time-dependent functions with static symbols
    final_position_expression = sp.simplify(end_effector_position_t.subs(symbol_mappings))
    final_jacobian_expression = sp.simplify(jacobian_symbolic_t.subs(symbol_mappings))
    final_acceleration_bias_expression = sp.simplify(acceleration_bias_t.subs(symbol_mappings))

    print("Symbolic computation completed successfully.")
    print("Creating optimized numerical functions via lambdification...")

    # Construct ordered symbol lists for consistent function signatures
    position_parameter_list = list(joint_symbols) + list(link_length_symbols) + list(link_offset_symbols)
    complete_parameter_list = position_parameter_list + list(velocity_symbols)

    # Generate high-performance numerical functions
    numerical_position_function = sp.lambdify(position_parameter_list, final_position_expression, 'numpy')
    numerical_jacobian_function = sp.lambdify(position_parameter_list, final_jacobian_expression, 'numpy')
    numerical_acceleration_bias_function = sp.lambdify(complete_parameter_list, final_acceleration_bias_expression, 'numpy')

    print("Numerical function generation completed successfully.")
    
    return (final_position_expression, final_jacobian_expression, numerical_position_function, 
            numerical_jacobian_function, numerical_acceleration_bias_function, position_parameter_list, complete_parameter_list)


class RobotKinematics:
    """Advanced kinematics processor for robotic manipulator analysis."""
    
    def __init__(self, link_lengths: Optional[List[float]] = None, link_offsets: Optional[List[float]] = None):
        """Configure robot manipulator with geometric parameters.
        
        Args:
            link_lengths: Manipulator link lengths [L1, L2, L3, L4]
            link_offsets: Manipulator link offsets [a1, a2, a3, a4]
        """
        self.manipulator_lengths = link_lengths or DEFAULT_L_VALUES.copy()
        self.manipulator_offsets = link_offsets or DEFAULT_A_VALUES.copy()
        
    def _prepare_position_parameters(self, joint_configuration: np.ndarray) -> Tuple[float, ...]:
        """Prepare parameter tuple for position computation."""
        return tuple(joint_configuration) + tuple(self.manipulator_lengths) + tuple(self.manipulator_offsets)  # type: ignore
    
    def _prepare_velocity_parameters(self, joint_configuration: np.ndarray, joint_rates: np.ndarray) -> Tuple[float, ...]:
        """Prepare parameter tuple for velocity computation."""
        return self._prepare_position_parameters(joint_configuration) + tuple(joint_rates)  # type: ignore
    
    def forward_position_kinematics(self, joint_angles_deg: List[float], 
                                  position_func: Callable) -> np.ndarray:
        """Calculate forward position kinematics.
        
        Args:
            joint_angles_deg: Joint angles in degrees
            position_func: Position calculation function
            
        Returns:
            End-effector position (3x1 array)
        """
        joint_angles_rad = np.deg2rad(joint_angles_deg)  # type: ignore
        parameters = self._prepare_position_parameters(joint_angles_rad)
        position = np.array(position_func(*parameters)).reshape(3, 1)
        return position
    
    def forward_velocity_kinematics(self, joint_angles_deg: List[float], 
                                  joint_velocities_deg: List[float],
                                  jacobian_func: Callable) -> np.ndarray:
        """Calculate forward velocity kinematics.
        
        Args:
            joint_angles_deg: Joint angles in degrees
            joint_velocities_deg: Joint velocities in deg/s
            jacobian_func: Jacobian calculation function
            
        Returns:
            End-effector linear velocity (3x1 array)
        """
        joint_angles_rad = np.deg2rad(joint_angles_deg)  # type: ignore
        joint_velocities_rad = np.deg2rad(joint_velocities_deg)  # type: ignore
        
        parameters = self._prepare_position_parameters(joint_angles_rad)
        jacobian = np.array(jacobian_func(*parameters)).astype(float)
        
        velocity = jacobian @ joint_velocities_rad.reshape(4, 1)
        return velocity
    
    def inverse_position_kinematics(self, target_position: np.ndarray, 
                                  initial_guess_deg: List[float],
                                  position_func: Callable, jacobian_func: Callable) -> np.ndarray:
        """Solve inverse position kinematics using Newton-Raphson method.
        
        Args:
            target_position: Desired end-effector position (3x1)
            initial_guess_deg: Initial joint angle guess in degrees
            position_func: Position calculation function
            jacobian_func: Jacobian calculation function
            
        Returns:
            Solution joint angles in radians
        """
        joint_angles = np.deg2rad(initial_guess_deg)  # type: ignore
        target = target_position.reshape(3, 1)
        
        for iteration in range(MAX_ITERATIONS):
            # Calculate current position and error
            args = self._prepare_position_parameters(joint_angles)
            current_position = np.array(position_func(*args)).reshape(3, 1)
            error = target - current_position
            
            # Check convergence
            if np.linalg.norm(error) < TOLERANCE:
                print(f"Converged in {iteration} iterations.")
                break
                
            # Calculate Jacobian and pseudo-inverse
            jacobian = np.array(jacobian_func(*args)).astype(float)
            jacobian_t = jacobian.T
            damping_matrix = (DAMPING_FACTOR ** 2) * np.eye(3)
            
            try:
                # Damped least squares pseudo-inverse
                pseudo_inverse = jacobian_t @ np.linalg.inv(jacobian @ jacobian_t + damping_matrix)
            except np.linalg.LinAlgError:
                pseudo_inverse = np.linalg.pinv(jacobian)
            
            # Update joint angles
            delta_q = (pseudo_inverse @ error).flatten()
            joint_angles += delta_q
            
            if iteration == MAX_ITERATIONS - 1:
                print("Warning: IK did not converge within maximum iterations.")
        
        return joint_angles
    
    def inverse_velocity_kinematics(self, target_velocity: np.ndarray,
                                  joint_angles: np.ndarray, jacobian_func: Callable) -> np.ndarray:
        """Solve inverse velocity kinematics.
        
        Args:
            target_velocity: Desired end-effector velocity (3x1)
            joint_angles: Current joint angles in radians
            jacobian_func: Jacobian calculation function
            
        Returns:
            Required joint velocities in rad/s
        """
        args = self._prepare_position_parameters(joint_angles)
        jacobian = np.array(jacobian_func(*args)).astype(float)
        
        joint_velocities = np.linalg.pinv(jacobian) @ target_velocity.reshape(3, 1)
        return joint_velocities.flatten()
    
    def inverse_acceleration_kinematics(self, target_acceleration: np.ndarray,
                                      joint_angles: np.ndarray, joint_velocities: np.ndarray,
                                      jacobian_func: Callable, acceleration_bias_func: Callable) -> np.ndarray:
        """Solve inverse acceleration kinematics.
        
        Args:
            target_acceleration: Desired end-effector acceleration (3x1)
            joint_angles: Current joint angles in radians
            joint_velocities: Current joint velocities in rad/s
            jacobian_func: Jacobian calculation function
            acceleration_bias_func: Acceleration bias calculation function
            
        Returns:
            Required joint accelerations in rad/s²
        """
        # Calculate Jacobian and bias term
        pos_args = self._prepare_position_parameters(joint_angles)
        jacobian = np.array(jacobian_func(*pos_args)).astype(float)
        
        vel_args = self._prepare_velocity_parameters(joint_angles, joint_velocities)
        bias_term = np.array(acceleration_bias_func(*vel_args)).reshape(3, 1)
        
        # Solve for joint accelerations
        jacobian_pinv = np.linalg.pinv(jacobian)
        target = target_acceleration.reshape(3, 1)
        joint_accelerations = (jacobian_pinv @ (target - bias_term)).flatten()
        
        return joint_accelerations


def run_kinematic_scenario(position_func: Callable, jacobian_func: Callable, 
                         acceleration_bias_func: Callable, position_symbols: List, 
                         velocity_symbols: List) -> None:
    """Run interactive kinematic scenario with user input.
    
    Args:
        position_func: Position calculation function
        jacobian_func: Jacobian calculation function  
        acceleration_bias_func: Acceleration bias calculation function
        position_symbols: Position symbols list
        velocity_symbols: Velocity symbols list
    """
    robot = RobotKinematics()
    
    print(f'\n--- Robot Parameters ---')
    print(f'Link Lengths (L1..L4): {robot.manipulator_lengths}')
    print(f'Link Offsets (a1..a4): {robot.manipulator_offsets}')

    # Default values for continuing to next sections
    solution_angles = np.zeros(DOF)
    solution_velocities = np.zeros(DOF)
    
    try:
        # Forward kinematics test
        print('\n--- Interactive Forward Kinematics (4 DOF) ---')
        print("Enter 4 joint angles in degrees to compute end-effector position (or 'next' to continue).")
        print("(Press Ctrl+C to skip to next section)")

        try:
            while True:
                user_input = input("Enter 4 joint angles (deg) or 'next': ").strip()
                if user_input.lower() == 'next':
                    break
                    
                try:
                    joint_angles = [float(x) for x in user_input.split()]
                    if len(joint_angles) != DOF:
                        print(f"Please provide exactly {DOF} angles; got {len(joint_angles)}.")
                        continue
                        
                    position = robot.forward_position_kinematics(joint_angles, position_func)
                    print(f"End-effector Position (m): {np.round(position.flatten(), 6)}")
                    
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                except Exception as e:
                    print(f"Error: {e}")
                    
        except KeyboardInterrupt:
            print("\nSkipping to next section...")

        # Forward velocity kinematics
        print('\n--- Interactive Forward Velocity Kinematics (4 DOF) ---')
        print("Enter joint angles (deg) and velocities (deg/s) to compute end-effector velocity (or 'next' to continue).")
        print("(Press Ctrl+C to skip to next section)")

        try:
            while True:
                angles_input = input("Enter 4 joint angles (deg) or 'next': ").strip()
                if angles_input.lower() == 'next':
                    break
                    
                try:
                    joint_angles = [float(x) for x in angles_input.split()]
                    if len(joint_angles) != DOF:
                        print(f"Expected {DOF} angles.")
                        continue
                        
                    velocities_input = input("Enter 4 joint velocities (deg/s): ").strip()
                    joint_velocities = [float(x) for x in velocities_input.split()]
                    if len(joint_velocities) != DOF:
                        print(f"Expected {DOF} velocities.")
                        continue

                    velocity = robot.forward_velocity_kinematics(joint_angles, joint_velocities, jacobian_func)
                    print(f"End-effector linear velocity (m/s): {np.round(velocity.flatten(), 6)}")
                    
                except ValueError:
                    print("Invalid input. Please enter numbers only.")
                except Exception as e:
                    print(f"Error: {e}")
                    
        except KeyboardInterrupt:
            print("\nSkipping to next section...")

        # Inverse position kinematics
        print('\n--- Inverse Position Kinematics (Newton-Raphson) ---')
        print("(Press Ctrl+C to skip to next section)")
        
        try:
            target_input = input('Enter desired end-effector position [x y z] in meters (example: "0.1 0 0.2"): ')
            target_position = np.array([float(val) for val in target_input.split()])

            guess_input = input('Provide starting joint angles [q1 q2 q3 q4] in degrees (example: "0 0 0 0"): ')
            initial_guess = [float(val) for val in guess_input.split()]

            solution_angles = robot.inverse_position_kinematics(
                target_position, initial_guess, position_func, jacobian_func)
            
            # Verify solution
            achieved_position = robot.forward_position_kinematics(
                np.rad2deg(solution_angles), position_func)  # type: ignore
            final_error = np.linalg.norm(target_position.reshape(3, 1) - achieved_position)

            print("\n--- IK Result ---")
            print(f"Target       : {np.round(target_position, 6)}")
            print(f"Achieved     : {np.round(achieved_position.flatten(), 6)}")
            print(f"Final error  : {final_error}")
            print(f"Solved q (deg): {np.round(np.rad2deg(solution_angles), 4)}")  # type: ignore
            
        except KeyboardInterrupt:
            print("\nSkipping to next section...")
            # Use default angles if user skipped this section
            solution_angles = np.zeros(DOF)

        # Inverse velocity kinematics
        print('\n--- Inverse Velocity Kinematics (IVK) ---')
        print("(Press Ctrl+C to skip to next section)")
        
        try:
            velocity_target_input = input('Specify desired end-effector velocity [vx vy vz] in m/s (example: "0.1 0 0"): ')
            target_velocity = np.array([float(val) for val in velocity_target_input.split()])

            solution_velocities = robot.inverse_velocity_kinematics(
                target_velocity, solution_angles, jacobian_func)
            
            print(f"Solved q_dot (deg/s): {np.round(np.rad2deg(solution_velocities), 4)}")  # type: ignore
            
        except KeyboardInterrupt:
            print("\nSkipping to next section...")
            # Use default velocities if user skipped this section
            solution_velocities = np.zeros(DOF)

        # Inverse acceleration kinematics
        print('\n--- Inverse Acceleration Kinematics ---')
        print("(Press Ctrl+C to skip to next section)")
        
        try:
            acceleration_target_input = input('Set target end-effector acceleration [ax ay az] in m/s² (example: "0 0 0"): ')
            target_acceleration = np.array([float(val) for val in acceleration_target_input.split()])

            solution_accelerations = robot.inverse_acceleration_kinematics(
                target_acceleration, solution_angles, solution_velocities,
                jacobian_func, acceleration_bias_func)
            
            print(f"Solved q_ddot (deg/s²): {np.round(np.rad2deg(solution_accelerations), 4)}")  # type: ignore

            # Verify acceleration solution
            pos_args = robot._prepare_position_parameters(solution_angles)
            jacobian = np.array(jacobian_func(*pos_args)).astype(float)
            
            vel_args = robot._prepare_velocity_parameters(solution_angles, solution_velocities)
            bias_term = np.array(acceleration_bias_func(*vel_args)).reshape(3, 1)
            
            verification = (jacobian @ solution_accelerations.reshape(4, 1)) + bias_term
            error_norm = np.linalg.norm(target_acceleration.reshape(3, 1) - verification)
            
            print(f"Verified X_ddot (m/s²): {np.round(verification.flatten(), 6)}")
            print(f"Acceleration error norm: {error_norm}")
            
        except KeyboardInterrupt:
            print("\nSkipping acceleration kinematics...")

        print("\n--- Scenario finished ---")

    except Exception as e:
        print(f"An error occurred in the main scenario: {e}")
        import traceback
        traceback.print_exc()


def main() -> None:
    """Primary application execution controller."""
    try:
        # Execute comprehensive symbolic kinematics computation
        (final_position_expr, final_jacobian_expr, numerical_pos_func, numerical_jac_func, 
         numerical_accel_bias_func, pos_param_list, vel_param_list) = symbolic_kinematics_calculation()

        # Present symbolic computation results
        print("\n=== Derived Forward Position Expression ===")
        sp.pprint(final_position_expr)
        print("\n=== Derived Jacobian Matrix Expression ===")  
        sp.pprint(final_jacobian_expr)

        # Execute interactive kinematic analysis scenario
        run_kinematic_scenario(numerical_pos_func, numerical_jac_func, numerical_accel_bias_func,
                             pos_param_list, vel_param_list)

    except KeyboardInterrupt:
        print("\nUser interrupted symbolic computation process.")
        print("Symbolic derivation is essential and cannot be bypassed. Terminating...")
        sys.exit(1)
    except Exception as error:
        print(f"Critical failure during symbolic computation initialization: {error}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()