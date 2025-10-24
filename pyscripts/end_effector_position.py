"""
End Effector Position Calculator
This script focuses specifically on calculating the end effector position
from joint angles using the forward kinematics equations.
"""

import sympy as sp
import numpy as np
from sympy import symbols, cos, sin, pi, simplify, lambdify
from kinematics import get_dh_parameters, create_joint_matrices, forward_kinematics


def get_forward_kinematics_equations():
    """
    Generate the symbolic forward kinematics equations for end effector position.
    
    Returns:
    --------
    tuple
        (x_equation, y_equation, z_equation) - symbolic expressions for position
    """
    
    print("Generating forward kinematics equations...")
    
    # Get DH parameters and create transformation matrices
    dh_params = get_dh_parameters()
    joint_matrices = create_joint_matrices(dh_params)
    
    # Calculate forward kinematics
    total_T, end_effector_pos = forward_kinematics(joint_matrices)
    
    return end_effector_pos


def create_numeric_functions(x_eq, y_eq, z_eq):
    """
    Convert symbolic equations to numeric functions for fast evaluation.
    
    Parameters:
    -----------
    x_eq, y_eq, z_eq : sympy expressions
        Symbolic expressions for x, y, z positions
    
    Returns:
    --------
    tuple
        (x_func, y_func, z_func) - numeric functions that can be called with joint angles
    """
    
    q1, q2, q3, q4 = symbols('q1 q2 q3 q4')
    
    # Create lambdified functions for fast numeric evaluation
    x_func = lambdify((q1, q2, q3, q4), x_eq, 'numpy')
    y_func = lambdify((q1, q2, q3, q4), y_eq, 'numpy')
    z_func = lambdify((q1, q2, q3, q4), z_eq, 'numpy')
    
    return x_func, y_func, z_func


def calculate_end_effector_position(q1_val, q2_val, q3_val, q4_val, equations=None):
    """
    Calculate end effector position for given joint angles.
    
    Parameters:
    -----------
    q1_val, q2_val, q3_val, q4_val : float
        Joint angle values in radians
    equations : tuple, optional
        Pre-calculated symbolic equations (x_eq, y_eq, z_eq)
    
    Returns:
    --------
    tuple
        (x, y, z) position of end effector
    """
    
    if equations is None:
        equations = get_forward_kinematics_equations()
    
    x_eq, y_eq, z_eq = equations
    
    # Substitute the joint angle values
    q1, q2, q3, q4 = symbols('q1 q2 q3 q4')
    
    x_val = float(x_eq.subs([(q1, q1_val), (q2, q2_val), (q3, q3_val), (q4, q4_val)]))
    y_val = float(y_eq.subs([(q1, q1_val), (q2, q2_val), (q3, q3_val), (q4, q4_val)]))
    z_val = float(z_eq.subs([(q1, q1_val), (q2, q2_val), (q3, q3_val), (q4, q4_val)]))
    
    return x_val, y_val, z_val


def test_end_effector_positions():
    """
    Test the end effector position calculation with various joint configurations.
    """
    
    print("=== Testing End Effector Position Calculation ===")
    
    # Get the symbolic equations once
    equations = get_forward_kinematics_equations()
    x_eq, y_eq, z_eq = equations
    
    print(f"\n=== Symbolic Forward Kinematics Equations ===")
    print(f"x(q1,q2,q3,q4) = {x_eq}")
    print(f"y(q1,q2,q3,q4) = {y_eq}")
    print(f"z(q1,q2,q3,q4) = {z_eq}")
    
    # Test configurations
    test_configs = [
        (0, 0, 0, 0),           # All joints at zero
        (pi/2, 0, 0, 0),        # First joint at 90 degrees
        (0, pi/2, 0, 0),        # Second joint at 90 degrees
        (0, 0, pi/2, 0),        # Third joint at 90 degrees
        (pi/4, pi/4, pi/4, pi/4), # All joints at 45 degrees
    ]
    
    print(f"\n=== Testing Different Joint Configurations ===")
    for i, (q1, q2, q3, q4) in enumerate(test_configs):
        print(f"\nConfiguration {i+1}: q1={q1:.3f}, q2={q2:.3f}, q3={q3:.3f}, q4={q4:.3f}")
        
        x, y, z = calculate_end_effector_position(q1, q2, q3, q4, equations)
        
        print(f"End effector position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        # Calculate distance from origin
        distance = np.sqrt(x**2 + y**2 + z**2)
        print(f"Distance from origin: {distance:.3f}")


def save_equations_to_file(equations, filename="forward_kinematics_equations.py"):
    """
    Save the forward kinematics equations to a Python file for later use.
    
    Parameters:
    -----------
    equations : tuple
        (x_eq, y_eq, z_eq) symbolic expressions
    filename : str
        Output filename
    """
    
    x_eq, y_eq, z_eq = equations
    
    content = f'''"""
Auto-generated forward kinematics equations for the 4-DOF robot arm.
Generated using DH parameters and symbolic computation.
"""

import sympy as sp
from sympy import symbols, cos, sin, pi

# Define joint variables
q1, q2, q3, q4 = symbols('q1 q2 q3 q4')

# Forward kinematics equations for end effector position
x_equation = {x_eq}

y_equation = {y_eq}

z_equation = {z_eq}

def get_equations():
    """Return the forward kinematics equations."""
    return x_equation, y_equation, z_equation

def evaluate_position(q1_val, q2_val, q3_val, q4_val):
    """
    Evaluate end effector position for given joint angles.
    
    Parameters:
    -----------
    q1_val, q2_val, q3_val, q4_val : float
        Joint angles in radians
    
    Returns:
    --------
    tuple : (x, y, z) position
    """
    x = float(x_equation.subs([(q1, q1_val), (q2, q2_val), (q3, q3_val), (q4, q4_val)]))
    y = float(y_equation.subs([(q1, q1_val), (q2, q2_val), (q3, q3_val), (q4, q4_val)]))
    z = float(z_equation.subs([(q1, q1_val), (q2, q2_val), (q3, q3_val), (q4, q4_val)]))
    
    return x, y, z
'''
    
    with open(filename, 'w') as f:
        f.write(content)
    
    print(f"Forward kinematics equations saved to {filename}")


if __name__ == "__main__":
    # Generate and test forward kinematics equations
    test_end_effector_positions()
    
    # Save equations to file
    equations = get_forward_kinematics_equations()
    save_equations_to_file(equations)