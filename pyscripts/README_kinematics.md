# Forward Kinematics Scripts for 4-DOF Robot Arm

This project contains Python scripts to calculate forward kinematics for a 4-DOF robot arm using DH (Denavit-Hartenberg) parameters.

## Files Created

### 1. `kinematics.py`
Core module containing the fundamental forward kinematics functions:
- `dh_transformation_matrix()`: Creates 4x4 transformation matrix from DH parameters
- `create_joint_matrices()`: Creates transformation matrices for all joints
- `forward_kinematics()`: Multiplies all joint matrices to get total transformation
- `get_dh_parameters()`: Returns the DH parameter table for your robot

### 2. `end_effector_position.py`
Specialized module for end effector position calculations:
- `get_forward_kinematics_equations()`: Generates symbolic forward kinematics equations
- `calculate_end_effector_position()`: Calculates position for given joint angles
- `test_end_effector_positions()`: Tests various joint configurations
- `save_equations_to_file()`: Saves equations to a Python file

### 3. `main.py`
Demonstration script that:
- Shows the complete forward kinematics calculation process
- Displays individual transformation matrices
- Shows the final forward kinematics equations
- Tests multiple joint configurations
- Provides detailed output with position calculations

## DH Parameters Used

| Joint |  θ    |    d    |    a    |   α   |
|-------|-------|---------|---------|-------|
|   1   |  q1   |  43.55  |   0.0   | -π/2  |
|   2   |  q2   |   0.0   | 140.0   |  0.0  |
|   3   |  q3   |   0.0   | 134.1   |  0.0  |
|   4   |  q4   |   0.0   |   0.0   |  0.0  |

Where:
- θ (theta): Joint angles (symbolic variables q1, q2, q3, q4)
- d: Link offset along z-axis
- a: Link length along x-axis  
- α (alpha): Link twist about x-axis

## Forward Kinematics Equations

The final forward kinematics equations for end effector position are:

```
x(q1,q2,q3,q4) = (140.0*cos(q2) + 134.1*cos(q2 + q3))*cos(q1)
y(q1,q2,q3,q4) = (140.0*cos(q2) + 134.1*cos(q2 + q3))*sin(q1)
z(q1,q2,q3,q4) = -140.0*sin(q2) - 134.1*sin(q2 + q3) + 43.55
```

Note: Joint q4 doesn't appear in the position equations because it only affects orientation (rotation about the end effector axis).

## Usage

### Run the complete demonstration:
```bash
uv run main.py
```

### Import modules in your own scripts:
```python
from kinematics import get_dh_parameters, create_joint_matrices, forward_kinematics
from end_effector_position import calculate_end_effector_position

# Calculate position for specific joint angles
x, y, z = calculate_end_effector_position(0, pi/4, pi/4, 0)
print(f"End effector position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
```

### Test individual modules:
```bash
uv run python kinematics.py
uv run python end_effector_position.py
```

## Dependencies

- NumPy: For numerical computations
- SymPy: For symbolic mathematics and equation manipulation

Both are already included in the `pyproject.toml` file.

## Key Features

1. **Symbolic Computation**: Uses SymPy to maintain exact symbolic expressions
2. **Modular Design**: Separate modules for different aspects of the calculation
3. **Comprehensive Testing**: Multiple test configurations to verify correctness
4. **Clear Output**: Detailed printing of matrices and equations
5. **Easy Integration**: Functions can be imported and used in other projects

## Mathematical Approach

1. **DH Parameters**: Define the geometric relationship between adjacent joints
2. **Transformation Matrices**: Each joint creates a 4x4 homogeneous transformation matrix
3. **Matrix Multiplication**: Total transformation = T1 × T2 × T3 × T4
4. **Position Extraction**: End effector position comes from the last column of the total transformation matrix

The resulting equations give you the (x, y, z) position of the end effector in terms of the joint angles (q1, q2, q3, q4).