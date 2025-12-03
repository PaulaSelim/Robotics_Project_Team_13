# Raspberry Pi 5 Robot Arm Setup Guide

## Hardware Setup

### Components
- Raspberry Pi 5 (with ROS Noetic)
- 3x MG996R Servo Motors
- 1x Micro Servo (gripper)
- External 6V/5A Power Supply
- Jumper wires

### GPIO Wiring (BCM Pin Numbering)

```
Raspberry Pi 5 GPIO:
┌────────────────────────────────┐
│ Pin Layout (BCM numbering)     │
├────────────────────────────────┤
│ GPIO 18 (Physical Pin 12) ─────┼──> Joint_1 Signal (MG996R)
│ GPIO 13 (Physical Pin 33) ─────┼──> Joint_2 Signal (MG996R)
│ GPIO 12 (Physical Pin 32) ─────┼──> Joint_3 Signal (MG996R)
│ GPIO 19 (Physical Pin 35) ─────┼──> Joint_5 Signal (Micro)
│                                │
│ GND (Pins 6,9,14,20,25,30...) ┼──> Common Ground
└────────────────────────────────┘

Physical Pin Reference:
  1 [3.3V]      2 [5V]
  3 [GPIO 2]    4 [5V]
  5 [GPIO 3]    6 [GND]  ←── Connect to power supply GND
  ...
 11 [GPIO 17]  12 [GPIO 18] ←── Joint 1
 ...
 31 [GPIO 6]   32 [GPIO 12] ←── Joint 3
 33 [GPIO 13]  34 [GND]     ←── Joint 2
 35 [GPIO 19]  36 [GPIO 16] ←── Joint 5
 ...

External 6V Power Supply:
  (+) 6V ──────> All Servo RED wires
  (-) GND ─────> All Servo BROWN/BLACK wires + Pi GND (Pin 6)

⚠️ IMPORTANT:
- NEVER connect servo power (red) to Pi 5V pins!
- Always use external regulated 6V supply (5-6A capacity)
- Common ground is MANDATORY
```

### Power Requirements

- **Pi 5**: 5V/3A via USB-C (separate from servo power)
- **Servos**: 6V/5A external supply
  - Each MG996R draws up to 1-1.5A when moving
  - Total: 3 × 1.5A + micro servo = ~5A peak
  - Use switching power supply rated for 6V/6A or higher

## Software Setup on Raspberry Pi 5

### Step 1: Clone Repository

```bash
# SSH into your Pi 5
ssh ubuntu@<pi5-ip-address>

# Clone your repository
cd ~
git clone https://github.com/PaulaSelim/Robotics_Project_Team_13.git
cd Robotics_Project_Team_13

# Or if already cloned, just pull latest
cd ~/Robotics_Project_Team_13
git checkout feature/milestone_3
git pull
```

### Step 2: Install pigpio Library

```bash
# Install pigpio for GPIO control
sudo apt update
sudo apt install pigpio python3-pigpio

# Enable and start pigpio daemon
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Verify it's running
sudo systemctl status pigpiod
```

Should show: `Active: active (running)`

### Step 3: Setup Catkin Workspace (if not in Docker)

If running natively (not in Docker):

```bash
# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/Robotics_Project_Team_13/src/my_robot_gazebo .
ln -s ~/Robotics_Project_Team_13/src/my_robot_moveit_config .
ln -s ~/Robotics_Project_Team_13/src/validation_pkg .

# Build workspace
cd ~/catkin_ws
catkin_make

# Source workspace
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### Step 4: Make Scripts Executable

```bash
cd ~/catkin_ws/src/my_robot_gazebo/scripts
chmod +x pi_servo_controller.py
chmod +x cubic_trajectory_planner.py
```

### Step 5: Test GPIO Access

```bash
# Test pigpio
python3 << EOF
import pigpio
pi = pigpio.pi()
if pi.connected:
    print("✅ pigpio connected successfully!")
    pi.stop()
else:
    print("❌ Failed to connect to pigpio daemon")
EOF
```

## Running the System

### Option A: Native ROS (Recommended)

If ROS is installed natively on Pi:

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Start Pi servo controller
source ~/catkin_ws/devel/setup.bash
rosrun my_robot_gazebo pi_servo_controller.py

# Terminal 3: Run trajectory planner (can run on Pi or your laptop)
source ~/catkin_ws/devel/setup.bash
rosrun my_robot_gazebo cubic_trajectory_planner.py
```

### Option B: ROS in Docker

If using Docker for ROS:

```bash
# Terminal 1: Start ROS core in Docker
docker exec -it <ros-container-name> roscore

# Terminal 2: Start Pi servo controller (OUTSIDE Docker - needs GPIO access)
source ~/catkin_ws/devel/setup.bash
./src/my_robot_gazebo/scripts/pi_servo_controller.py

# Terminal 3: Trajectory planner in Docker
docker exec -it <ros-container-name> bash
source /opt/ros/noetic/setup.bash
cd /workspace/catkin_ws
source devel/setup.bash
rosrun my_robot_gazebo cubic_trajectory_planner.py
```

**Note**: Servo controller MUST run outside Docker to access GPIO!

### Option C: Network Setup (Control from Your PC)

Run trajectory planning on your laptop, control Pi servos over network:

**On Raspberry Pi 5:**
```bash
# Get Pi IP address
hostname -I
# Example: 192.168.1.100

# Set ROS_MASTER_URI (roscore runs on your PC)
export ROS_MASTER_URI=http://192.168.1.50:11311  # Your PC's IP
export ROS_IP=192.168.1.100  # Pi's IP

# Start servo controller
sudo pigpiod  # Ensure daemon is running
./pi_servo_controller.py
```

**On Your PC:**
```bash
# Set environment variables
export ROS_MASTER_URI=http://192.168.1.50:11311  # Your PC's IP
export ROS_IP=192.168.1.50

# Start roscore
roscore

# In another terminal: Run trajectory planner
source ~/catkin_ws/devel/setup.bash
rosrun my_robot_gazebo cubic_trajectory_planner.py
```

Both machines will communicate over network!

## Testing

### Test 1: Verify Topics

```bash
# List topics
rostopic list

# Should see:
# /Joint_1/command
# /Joint_2/command
# /Joint_3/command
# /Joint_4/command
# /Joint_5/command

# Check servo controller is subscribed
rostopic info /Joint_1/command
```

### Test 2: Manual Servo Test

```bash
# Move Joint 1 to center (0 radians = 90 degrees)
rostopic pub /Joint_1/command std_msgs/Float64 "data: 0.0" --once

# Move Joint 1 to +45 degrees (0.785 radians)
rostopic pub /Joint_1/command std_msgs/Float64 "data: 0.785" --once

# Move Joint 1 to -45 degrees (-0.785 radians)
rostopic pub /Joint_1/command std_msgs/Float64 "data: -0.785" --once
```

Servo should move smoothly!

### Test 3: Run Example Trajectory

```bash
# Run trajectory planner and select option 2 (example)
rosrun my_robot_gazebo cubic_trajectory_planner.py
```

## Advantages vs Arduino

| Feature | Arduino | Raspberry Pi 5 |
|---------|---------|----------------|
| **ROS Integration** | rosserial (limited) | Native ROS node ✅ |
| **Processing Power** | 16 MHz | Quad-core 2.4 GHz ✅ |
| **Memory** | 2 KB RAM | 4-8 GB RAM ✅ |
| **Programming** | C++ only | Python, C++, any language ✅ |
| **Debugging** | Serial monitor only | Full ROS tools, rviz, rqt ✅ |
| **Network** | None | WiFi, Ethernet ✅ |
| **Can Run Trajectory Planner** | ❌ No | ✅ Yes |
| **Setup Complexity** | Simpler | More complex |
| **Cost** | ~$25 | ~$80 |

## Troubleshooting

### pigpio Daemon Not Running

```bash
# Check status
sudo systemctl status pigpiod

# If not running:
sudo systemctl start pigpiod

# Enable on boot
sudo systemctl enable pigpiod
```

### Permission Denied Errors

```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER

# Reboot
sudo reboot
```

### Servos Jittering

- Check power supply is adequate (6V/5A+)
- Verify common ground connection
- Reduce update frequency in code
- Add capacitor (1000µF) across power supply

### ROS Network Issues

```bash
# Verify connectivity
ping <other-machine-ip>

# Check ROS_MASTER_URI and ROS_IP
echo $ROS_MASTER_URI
echo $ROS_IP

# Test ROS connectivity
rosnode list
```

### Servo Not Moving

```bash
# Test GPIO directly
python3 << EOF
import pigpio
import time
pi = pigpio.pi()
pin = 18  # Joint 1
pi.set_servo_pulsewidth(pin, 1500)  # Center
time.sleep(1)
pi.set_servo_pulsewidth(pin, 1000)  # One direction
time.sleep(1)
pi.set_servo_pulsewidth(pin, 2000)  # Other direction
time.sleep(1)
pi.set_servo_pulsewidth(pin, 0)     # Off
pi.stop()
EOF
```

## Performance Optimization

### For Better Performance:

```bash
# Increase process priority
sudo renice -n -10 -p $(pgrep -f pi_servo_controller)

# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable cups

# Reduce GPU memory (edit /boot/firmware/config.txt)
sudo nano /boot/firmware/config.txt
# Add: gpu_mem=16

# Reboot
sudo reboot
```

## Files Structure

```
catkin_ws/
├── src/
│   └── my_robot_gazebo/
│       └── scripts/
│           ├── pi_servo_controller.py      # New! Pi GPIO controller
│           ├── cubic_trajectory_planner.py  # Unchanged
│           ├── inverseKinematics.py         # Unchanged
│           └── arduino_test/
│               └── arduino_test.ino         # Not needed for Pi
```

## Quick Start Checklist

- [ ] Hardware wired correctly (check GPIO pins!)
- [ ] External power supply connected (6V/5A)
- [ ] Common ground between Pi and power supply
- [ ] pigpiod installed and running
- [ ] Repository cloned to Pi
- [ ] Scripts made executable
- [ ] ROS environment sourced
- [ ] roscore running
- [ ] pi_servo_controller.py running
- [ ] Test with `rostopic pub` works
- [ ] Ready to run trajectory planner!

## Safety Notes

⚠️ **IMPORTANT:**
1. Never connect servo power to Pi 5V pins - will damage Pi!
2. Always common ground between Pi and power supply
3. Use proper 6V regulated power supply
4. Test one servo at a time initially
5. Keep emergency power disconnect ready
6. Monitor servo temperature during extended use
