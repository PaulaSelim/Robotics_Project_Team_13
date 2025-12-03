# Arduino Robot Arm Setup Guide (ROS Integration)

## Hardware Setup

### Components Required
- Arduino Uno/Mega
- 3x MG996R Servo Motors (for Joint 1, 2, 3)
- 1x Micro Servo (for Joint 5 - gripper)
- External 5-6V Power Supply (IMPORTANT: Servos draw too much current for Arduino!)
- Jumper wires

### Wiring Connections

```
Arduino Pin Connections:
┌─────────────────────┐
│   Arduino Board     │
│                     │
│  Pin 9  ────────────┼──> Joint_1 (MG996R) Signal Wire (Orange/Yellow)
│  Pin 10 ────────────┼──> Joint_2 (MG996R) Signal Wire (Orange/Yellow)
│  Pin 11 ────────────┼──> Joint_3 (MG996R) Signal Wire (Orange/Yellow)
│  Pin 6  ────────────┼──> Joint_5 (Micro)  Signal Wire (Orange/Yellow)
│                     │
│  GND ───────────────┼──> Common Ground (connect to power supply GND)
└─────────────────────┘

External Power Supply (5-6V):
┌─────────────────────┐
│  Power Supply       │
│                     │
│  (+) VCC ───────────┼──> All Servo RED wires
│                     │
│  (-) GND ───────────┼──> All Servo BROWN wires + Arduino GND
└─────────────────────┘

⚠️  IMPORTANT: Connect Arduino GND to Power Supply GND (common ground)!
```

### Servo Wire Colors
- **RED** = Power (5-6V from external supply)
- **BROWN/BLACK** = Ground (GND)
- **ORANGE/YELLOW** = Signal (to Arduino pins)

## Software Setup

### Step 1: Install rosserial Packages

```bash
# Install rosserial for Arduino
sudo apt install ros-noetic-rosserial-arduino ros-noetic-rosserial
```

### Step 2: Install ROS Library in Arduino IDE

```bash
# Navigate to Arduino libraries directory
cd ~/Arduino/libraries

# Remove old version if exists
rm -rf ros_lib

# Generate new ROS library for Arduino
rosrun rosserial_arduino make_libraries.py .
```

You should see output like:
```
Exporting to ~/Arduino/libraries/ros_lib
Exporting std_msgs
Exporting sensor_msgs
...
```

### Step 3: Upload Arduino Code

1. Open Arduino IDE
2. Open the file: `~/catkin_ws/src/my_robot_gazebo/scripts/arduino_test/arduino_test.ino`
3. Select your board: **Tools → Board → Arduino Uno** (or your model)
4. Select port: **Tools → Port → /dev/ttyACM1** (or detected port)
5. Click **Upload** button (→)
6. Wait for "Done uploading" message

### Step 4: Verify Arduino Code Compiled

If you get compilation errors about `ros.h`, make sure you ran Step 2 correctly.

Common issues:
- `ros.h: No such file or directory` → Run `rosrun rosserial_arduino make_libraries.py .` in `~/Arduino/libraries`
- Library not found → Check `~/Arduino/libraries/ros_lib` exists

## Running the System

### Terminal 1: Start roscore

```bash
roscore
```

### Terminal 2: Connect Arduino to ROS (rosserial bridge)

```bash
# Make sure Arduino is plugged in and uploaded
# Check port: ls -l /dev/ttyACM*

# Run rosserial bridge
rosrun rosserial_python serial_node.py /dev/ttyACM1
```

You should see:
```
[INFO] [1701547890.123456]: ROS Serial Python Node
[INFO] [1701547890.234567]: Connecting to /dev/ttyACM1 at 57600 baud
[INFO] [1701547892.345678]: Requesting topics...
[INFO] [1701547892.456789]: Note: subscribe buffer size is 512 bytes
[INFO] [1701547892.567890]: Setup subscriber on /Joint_1/command [std_msgs/Float64]
[INFO] [1701547892.678901]: Setup subscriber on /Joint_2/command [std_msgs/Float64]
[INFO] [1701547892.789012]: Setup subscriber on /Joint_3/command [std_msgs/Float64]
[INFO] [1701547892.890123]: Setup subscriber on /Joint_5/command [std_msgs/Float64]
```

### Terminal 3: (Optional) Launch Gazebo Simulation

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch my_robot_gazebo my_robot_world.launch
```

### Terminal 4: Run Trajectory Planner

```bash
cd ~/catkin_ws/src/my_robot_gazebo/scripts
python3 cubic_trajectory_planner.py
```

## Testing

### Test 1: Verify ROS Topics

In a new terminal:
```bash
# Check if Arduino subscribers are active
rostopic list

# You should see:
# /Joint_1/command
# /Joint_2/command
# /Joint_3/command
# /Joint_4/command
# /Joint_5/command

# Check topic info
rostopic info /Joint_1/command

# Should show Arduino node as subscriber
```

### Test 2: Manual Joint Command

Send a test command:
```bash
# Move Joint 1 to 45 degrees (0.785 radians)
rostopic pub /Joint_1/command std_msgs/Float64 "data: 0.785" --once

# The Arduino servo should move!
```

### Test 3: Run Example Trajectory

In the trajectory planner, choose option 2 to run the example trajectory. Both Gazebo (if running) and the physical robot should move simultaneously!

## Troubleshooting

### Serial Node Can't Connect

```bash
# Check permissions
ls -l /dev/ttyACM1

# Should show: crw-rw-rw- 1 root dialout
# If not, run:
sudo chmod 666 /dev/ttyACM1

# Permanent fix:
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### Arduino Not Responding

1. Check Serial Monitor in Arduino IDE (115200 baud) - should show nothing (ROS controls it)
2. Verify rosserial bridge is running and shows "Setup subscriber" messages
3. Check USB cable is good quality (data, not just power)
4. Try unplugging/replugging Arduino

### Servos Not Moving

1. **Check power supply**: Servos MUST have external 5-6V power
2. **Common ground**: Arduino GND must connect to power supply GND
3. **Check rostopic echo**: `rostopic echo /Joint_1/command` to see if commands are being sent
4. **LED blinking**: Arduino LED should blink when receiving commands

### Compilation Errors

**Error**: `ros.h: No such file or directory`
```bash
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```

**Error**: `Servo.h: No such file or directory`  
Servo library should be built-in. Try reinstalling Arduino IDE.

### rosserial_python Not Found

```bash
sudo apt install ros-noetic-rosserial-python
```

## How It Works

### Architecture

```
┌─────────────────────┐
│  Python Script      │  ROS Node (trajectory planner)
│  cubic_traj...py    │  - Calculates trajectories
└──────────┬──────────┘  - Publishes to /Joint_X/command topics
           │
           │ ROS Topics
           │ (std_msgs/Float64)
           │
     ┌─────┴─────┬──────────────┐
     │           │              │
     ▼           ▼              ▼
┌─────────┐ ┌─────────┐  ┌──────────────┐
│ Gazebo  │ │rosserial│  │   Arduino    │
│Controllers  │ bridge  │  │   (ROS node) │
└─────────┘ └────┬────┘  └──────────────┘
                 │ Serial (USB)
                 │
            ┌────▼────┐
            │ Arduino │  Physical Hardware
            │  Board  │  - Receives Float64 messages
            └────┬────┘  - Converts to servo commands
                 │
        ┌────────┴────────┬────────┐
        ▼                 ▼        ▼
    Servo 1           Servo 2  Servo 3,5
```

### Message Flow

1. **Trajectory Planner** calculates next position
2. Publishes `Float64` messages to `/Joint_X/command` topics
3. **Gazebo** controllers receive and move simulated robot
4. **rosserial bridge** forwards messages over USB serial
5. **Arduino** receives messages via ROS callbacks
6. Converts radians to degrees and moves physical servos

## Advantages of ROS Method

✅ **Single source of truth**: One topic controls both sim and hardware  
✅ **Synchronization**: Both robots move together automatically  
✅ **ROS ecosystem**: Can use rviz, rqt, rosbag, etc.  
✅ **Easy debugging**: `rostopic echo` to see all commands  
✅ **Scalable**: Easy to add more subscribers or controllers  

## File Locations

- **Arduino code**: `/home/paula/catkin_ws/src/my_robot_gazebo/scripts/arduino_test/arduino_test.ino`
- **Python script**: `/home/paula/catkin_ws/src/my_robot_gazebo/scripts/cubic_trajectory_planner.py`
- **ROS library**: `~/Arduino/libraries/ros_lib/`

## Safety Notes

⚠️  **IMPORTANT SAFETY WARNINGS:**

1. **Never power servos from Arduino 5V pin!** Use external supply.
2. **Always connect common ground** between Arduino and power supply
3. **Start with small movements** to test mechanical limits
4. **Watch for collisions** with robot structure
5. **Keep hands clear** during operation
6. **Emergency stop**: Ctrl+C in rosserial terminal or unplug power

## Hardware Setup

### Components Required
- Arduino Uno/Mega
- 3x MG996R Servo Motors (for Joint 1, 2, 3)
- 1x Micro Servo (for Joint 5 - gripper)
- External 5-6V Power Supply (IMPORTANT: Servos draw too much current for Arduino!)
- Jumper wires

### Wiring Connections

```
Arduino Pin Connections:
┌─────────────────────┐
│   Arduino Board     │
│                     │
│  Pin 9  ────────────┼──> Joint_1 (MG996R) Signal Wire (Orange/Yellow)
│  Pin 10 ────────────┼──> Joint_2 (MG996R) Signal Wire (Orange/Yellow)
│  Pin 11 ────────────┼──> Joint_3 (MG996R) Signal Wire (Orange/Yellow)
│  Pin 6  ────────────┼──> Joint_5 (Micro)  Signal Wire (Orange/Yellow)
│                     │
│  GND ───────────────┼──> Common Ground (connect to power supply GND)
└─────────────────────┘

External Power Supply (5-6V):
┌─────────────────────┐
│  Power Supply       │
│                     │
│  (+) VCC ───────────┼──> All Servo RED wires
│                     │
│  (-) GND ───────────┼──> All Servo BROWN wires + Arduino GND
└─────────────────────┘

⚠️  IMPORTANT: Connect Arduino GND to Power Supply GND (common ground)!
```

### Servo Wire Colors
- **RED** = Power (5-6V from external supply)
- **BROWN/BLACK** = Ground (GND)
- **ORANGE/YELLOW** = Signal (to Arduino pins)

## Software Setup

### Step 1: Fix Serial Port Permissions

```bash
# Method 1: Add user to dialout group (permanent fix)
sudo usermod -a -G dialout $USER

# Then log out and log back in (or reboot)

# Method 2: Quick temporary fix
sudo chmod 666 /dev/ttyACM*
sudo chmod 666 /dev/ttyUSB*
```

### Step 2: Upload Arduino Code

1. Open Arduino IDE
2. Open the file: `arduino_robot_controller.ino`
3. Select your board: **Tools → Board → Arduino Uno** (or your model)
4. Select port: **Tools → Port → /dev/ttyACM0** (or detected port)
5. Click **Upload** button (→)
6. Wait for "Done uploading" message

### Step 3: Verify Arduino is Working

Open **Tools → Serial Monitor** and set to **115200 baud**. You should see:
```
===========================================
4-DOF Robot Arm Controller Ready
===========================================
Waiting for commands...
Format: J1:angle1,J2:angle2,J3:angle3,J5:angle5
===========================================
```

### Step 4: Install Python Dependencies

```bash
# Install pyserial for Arduino communication
pip3 install pyserial
```

### Step 5: Run the Trajectory Planner

```bash
# Navigate to scripts directory
cd ~/catkin_ws/src/my_robot_gazebo/scripts

# Make script executable
chmod +x cubic_trajectory_planner.py

# Run the script
python3 cubic_trajectory_planner.py

# Or with ROS:
rosrun my_robot_gazebo cubic_trajectory_planner.py
```

## Usage

When you run the Python script:

1. It will ask: **"Connect to Arduino hardware? (y/n, default=y):"**
   - Press `y` or Enter to connect to Arduino
   - Press `n` for simulation only

2. If Arduino is found, you'll see:
   ```
   🔍 Searching for Arduino...
   ✅ Found: /dev/ttyACM0 - Arduino Uno
   🔌 Connecting to Arduino...
   ✅ Arduino connected successfully!
   ```

3. Choose operation mode:
   - **1** = Interactive mode (enter custom positions)
   - **2** = Run example trajectory
   - **3** = Exit

4. The robot will move **simultaneously** in:
   - Gazebo simulation (if running)
   - Physical Arduino robot arm

## Testing

### Quick Test in Serial Monitor

In Arduino IDE Serial Monitor (115200 baud), type:
```
J1:0.0,J2:0.0,J3:0.0,J5:0.0
```
All servos should move to center position (90°).

Try:
```
J1:45.0,J2:-30.0,J3:60.0,J5:90.0
```
Servos should move to these angles.

## Troubleshooting

### Permission Denied Error
```bash
sudo chmod 666 /dev/ttyACM0
# Or permanently:
sudo usermod -a -G dialout $USER
# Then logout and login
```

### Arduino Not Detected
- Check USB cable connection
- Try different USB port
- Check if Arduino shows up: `ls -l /dev/ttyACM* /dev/ttyUSB*`
- Check dmesg: `dmesg | grep tty | tail -5`

### Servos Not Moving
- Check power supply is connected and ON
- Verify servo wires are properly connected
- Check Serial Monitor for error messages
- Ensure common ground between Arduino and power supply

### Servos Jittering
- Use external power supply (Arduino 5V is not enough!)
- Add capacitor (100-1000µF) across power supply terminals
- Check for loose connections

## Protocol Details

### Serial Communication Format
```
Command: "J1:angle1,J2:angle2,J3:angle3,J5:angle5\n"
- Angles in DEGREES
- Range: -180° to +180°
- Baud rate: 115200
- Line ending: \n (newline)
```

### Example Commands
```
J1:0.0,J2:0.0,J3:0.0,J5:0.0       # Center all servos
J1:45.0,J2:-30.0,J3:60.0,J5:90.0  # Mixed angles
J1:90.0,J2:0.0,J3:0.0,J5:0.0      # Rotate base only
```

## Safety Notes

⚠️  **IMPORTANT SAFETY WARNINGS:**

1. **Never power servos from Arduino 5V pin!** Use external supply.
2. **Always connect common ground** between Arduino and power supply
3. **Start with small movements** to test mechanical limits
4. **Watch for collisions** with robot structure
5. **Keep hands clear** during operation
6. **Emergency stop:** Unplug power supply immediately if needed

## Adjusting Servo Limits

Edit the Arduino code to match your physical robot constraints:

```cpp
// Servo limits (adjust based on your physical constraints)
const float SERVO_MIN[4] = {0, 10, 20, 0};    // Minimum angles
const float SERVO_MAX[4] = {180, 170, 160, 180};  // Maximum angles
```

Test carefully after any changes!

## File Locations

- Arduino code: `/home/paula/catkin_ws/arduino_robot_controller.ino`
- Python script: `/home/paula/catkin_ws/src/my_robot_gazebo/scripts/cubic_trajectory_planner.py`
