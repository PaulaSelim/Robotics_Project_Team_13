# Terminal Input Options for Joint Angles

You now have several ways to input joint angles from the terminal:

## Method 1: Enhanced Interactive Main Script

Run the main script and choose interactive mode:

```bash
uv run main.py
# Choose option 2 or 3 for interactive mode
# Supports both degrees and radians input
# Continuous calculations with helpful prompts
```

## Method 2: Command-Line Calculator (Recommended)

Use the dedicated command-line tool:

```bash
# Basic usage (angles in radians)
uv run python calculate_position.py 0 0 0 0
uv run python calculate_position.py 1.57 0 0 0      # q1 at 90°
uv run python calculate_position.py 0.785 0.785 0 0 # q1,q2 at 45°

# Using degrees
uv run python calculate_position.py --degrees 0 0 0 0
uv run python calculate_position.py --degrees 90 0 0 0
uv run python calculate_position.py -d 45 45 0 0

# Quiet mode (only outputs x y z coordinates - good for scripting)
uv run python calculate_position.py --quiet 0 0 0 0
uv run python calculate_position.py -q -d 90 45 0 0

# Help
uv run python calculate_position.py --help
```

## Method 3: Direct Function Import

For integration into other Python scripts:

```python
from end_effector_position import calculate_end_effector_position

# Calculate position for specific angles (in radians)
x, y, z = calculate_end_effector_position(0, 1.57, 0, 0)
print(f"Position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
```

## Examples

### Command-Line Examples:

```bash
# Home position (all joints at zero)
uv run python calculate_position.py 0 0 0 0

# First joint at 90 degrees
uv run python calculate_position.py --degrees 90 0 0 0

# Arm extended forward and up
uv run python calculate_position.py --degrees 0 45 -45 0

# Complex configuration
uv run python calculate_position.py --degrees 45 30 60 0

# Get just the coordinates for use in other scripts
POSITION=$(uv run python calculate_position.py --quiet 0 1.57 0 0)
echo "End effector is at: $POSITION"
```

### Expected Outputs:

```bash
# Home position output:
End effector position:
  x = 0.273000
  y = 0.000000  
  z = 0.049750

# With 90° first joint:
End effector position:
  x = 0.000000
  y = 0.273000
  z = 0.049750
```

## Features

✅ **Multiple Input Formats**: Radians or degrees
✅ **Command-Line Arguments**: Direct angle input via command line
✅ **Interactive Mode**: Step-by-step angle input with prompts
✅ **Quiet Mode**: Minimal output for scripting integration
✅ **Detailed Output**: Joint angles, position, distances, angles
✅ **Error Handling**: Validates input and provides helpful error messages
✅ **Help Documentation**: Built-in help with examples

## File Purposes

- **`main.py`**: Interactive demonstration with menu system
- **`calculate_position.py`**: Command-line tool (recommended for terminal use)
- **`angle_input.py`**: Alternative interactive script
- **`kinematics.py`**: Core transformation matrix functions
- **`end_effector_position.py`**: Position calculation functions

The **`calculate_position.py`** script is recommended for terminal input as it provides the most flexible and user-friendly command-line interface.