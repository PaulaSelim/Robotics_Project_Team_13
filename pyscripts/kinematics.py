"""
Kinematics module for robot forward kinematics using DH parameters.
This module provides functions to create transformation matrices and calculate
forward kinematics for a robotic arm.
"""

import sympy as sp
import numpy as np
from sympy import symbols, cos, sin, pi, Matrix, simplify


def dh_transformation_matrix(theta, d, a, alpha):
    """
    Create a 4x4 homogeneous transformation matrix using DH parameters.
    
    Parameters:
    -----------
    theta : sympy expression or float
        Joint angle (rotation about z-axis)
    d : float
        Link offset (translation along z-axis)
    a : float
        Link length (translation along x-axis)
    alpha : float
        Link twist (rotation about x-axis)
    
    Returns:
    --------
    sympy.Matrix
        4x4 homogeneous transformation matrix
    """
    
    # Create the transformation matrix using DH convention
    # T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
    
    T = Matrix([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)], # type: ignore
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)], # type: ignore
        [0,           sin(alpha),             cos(alpha),            d           ],
        [0,           0,                      0,                     1           ]
    ])
    
    return T


def create_joint_matrices(dh_params):
    """
    Create transformation matrices for all joints using DH parameters.
    
    Parameters:
    -----------
    dh_params : list
        List of tuples containing DH parameters (theta, d, a, alpha) for each joint
    
    Returns:
    --------
    list
        List of 4x4 transformation matrices
    """
    
    joint_matrices = []
    
    for i, (theta, d, a, alpha) in enumerate(dh_params, 1):
        print(f"Creating transformation matrix for Joint {i}:")
        print(f"  θ = {theta}, d = {d}, a = {a}, α = {alpha}")
        
        T = dh_transformation_matrix(theta, d, a, alpha)
        joint_matrices.append(T)
        
        print(f"T{i} =")
        sp.pprint(T)
        print()
    
    return joint_matrices


def forward_kinematics(joint_matrices):
    """
    Calculate forward kinematics by multiplying all joint transformation matrices.
    
    Parameters:
    -----------
    joint_matrices : list
        List of 4x4 transformation matrices
    
    Returns:
    --------
    tuple
        (total_transformation_matrix, end_effector_position)
        where end_effector_position is (x, y, z)
    """
    
    print("Calculating forward kinematics...")
    print("Multiplying transformation matrices: T_total = T1 * T2 * T3 * T4 * T5")
    
    # Start with identity matrix or first joint matrix
    total_T = joint_matrices[0]
    
    # Multiply all subsequent matrices
    for i in range(1, len(joint_matrices)):
        print(f"Multiplying with T{i+1}...")
        total_T = total_T * joint_matrices[i]
        total_T = simplify(total_T)  # Simplify after each multiplication
    
    print("Total transformation matrix:")
    sp.pprint(total_T)
    print()
    
    # Extract end effector position (x, y, z) from the last column
    end_effector_position = (
        total_T[0, 3],  # x
        total_T[1, 3],  # y
        total_T[2, 3]   # z
    )
    
    print("End effector position:")
    print(f"x = {end_effector_position[0]}")
    print(f"y = {end_effector_position[1]}")
    print(f"z = {end_effector_position[2]}")
    
    return total_T, end_effector_position


def calculate_forward_kinematics(joint_angles):
    """
    Calculate forward kinematics for given joint angles and return numeric values.
    
    Parameters:
    -----------
    joint_angles : list
        List of 5 joint angles [q1, q2, q3, q4, q5] in radians
    
    Returns:
    --------
    tuple
        (x, y, z) position of end effector as float values
    """
    
    print("Calculating forward kinematics for joint angles...")
    
    # Get DH parameters
    dh_params = get_dh_parameters()
    
    # Create transformation matrices for each joint
    joint_matrices = create_joint_matrices(dh_params)
    
    # Define symbolic joint variables
    q1, q2, q3, q4, q5 = symbols('q1 q2 q3 q4 q5')
    
    # Create substitution dictionary
    subs_dict = {
        q1: joint_angles[0],
        q2: joint_angles[1], 
        q3: joint_angles[2],
        q4: joint_angles[3],
        q5: joint_angles[4]
    }
    
    # Substitute actual joint values
    print("Substituting joint values...")
    numeric_matrices = []
    for i, matrix in enumerate(joint_matrices):
        print(f"Processing joint {i+1} matrix...")
        numeric_matrix = matrix.subs(subs_dict)
        numeric_matrices.append(numeric_matrix)
    
    # Calculate the total transformation matrix using numeric matrices
    print("Multiplying transformation matrices: T_total = T1 * T2 * T3 * T4 * T5")
    total_T = numeric_matrices[0]
    
    # Multiply all subsequent matrices using the numeric matrices
    for i in range(1, len(numeric_matrices)):
        print(f"Multiplying with T{i+1}...")
        total_T = total_T * numeric_matrices[i]
        total_T = simplify(total_T)  # Simplify after each multiplication
    
    print("Total transformation matrix:")
    sp.pprint(total_T)
    print()
    
    # Extract end effector position (x, y, z) from the last column and convert to float
    x_pos = float(total_T[0, 3])
    y_pos = float(total_T[1, 3])
    z_pos = float(total_T[2, 3])
    
    print("End effector position:")
    print(f"x = {x_pos}")
    print(f"y = {y_pos}")
    print(f"z = {z_pos}")
    
    return x_pos, y_pos, z_pos


def get_dh_parameters():
    """
    Define the DH parameters for the 5-DOF robot arm according to the URDF.
    
    Based on URDF analysis:
    - Joint_1: Base rotation (z-axis)
    - Joint_2: Shoulder pitch, offset by 0.04975m in z, rotated 90° about x
    - Joint_3: Elbow pitch, 0.140m link length
    - Joint_4: Wrist pitch, rotation changes
    - Joint_5: Wrist roll, final tool orientation
    
    Returns:
    --------
    list
        List of (theta, d, a, alpha) tuples for each joint
    """
    
    # Define symbolic joint variables for 5-DOF robot
    q1, q2, q3, q4, q5 = symbols('q1 q2 q3 q4 q5')
    
    # DH parameters extracted from URDF joint origins
    # Joint |  θ    |    d     |    a    |   α   
    # ------|-------|----------|---------|-------
    #   1   |  q1   |  0.0     |   0.0   |  0.0   (base rotation)
    #   2   |  q2   | 0.04975  |   0.0   | π/2    (shoulder, z-offset + x-rotation)
    #   3   |  q3   |  0.0     |  0.140  |  0.0   (elbow, link length)
    #   4   |  q4   |  0.0     |   0.0   | π/2    (wrist pitch)
    #   5   |  q5   | 0.13289  |   0.0   |  0.0   (wrist roll, final z-offset)
    
    dh_params = [
        (q1, 0.0, 0.0, 0.0),        # Joint 1: Base rotation
        (q2, 0.04975, 0.0, pi/2),   # Joint 2: Shoulder pitch with z-offset and x-rotation
        (q3, 0.0, 0.140, 0.0),      # Joint 3: Elbow pitch with link length
        (q4, 0.0, 0.0, pi/2),       # Joint 4: Wrist pitch with x-rotation
        (q5, 0.13289, 0.0, 0.0),    # Joint 5: Wrist roll with final z-offset
    ]
    
    return dh_params


def get_symbolic_forward_kinematics():
    """
    Calculate symbolic forward kinematics equations.
    
    Returns:
    --------
    tuple
        (transformation_matrix, position_equations)
        where position_equations is (x_eq, y_eq, z_eq)
    """
    
    print("Calculating symbolic forward kinematics...")
    
    # Get DH parameters
    dh_params = get_dh_parameters()
    
    # Create joint matrices
    joint_matrices = create_joint_matrices(dh_params)
    
    # Calculate forward kinematics
    total_T, end_effector_equations = forward_kinematics(joint_matrices)
    
    return total_T, end_effector_equations


if __name__ == "__main__":
    # Example usage
    print("Forward Kinematics Module")
    print("="*40)
    
    # Calculate symbolic equations
    transformation_matrix, equations = get_symbolic_forward_kinematics()
    
    print("\nSymbolic forward kinematics equations:")
    print(f"x(q1,q2,q3,q4,q5) = {equations[0]}")
    print(f"y(q1,q2,q3,q4,q5) = {equations[1]}")
    print(f"z(q1,q2,q3,q4,q5) = {equations[2]}")
    
    # Test with specific joint angles
    print("\n" + "="*40)
    print("Testing with zero joint angles:")
    
    test_angles = [0, 0, 0, 0, 0]
    x, y, z = calculate_forward_kinematics(test_angles)
    
    print(f"Result: x={x}, y={y}, z={z}")