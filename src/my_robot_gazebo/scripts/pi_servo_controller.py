#!/usr/bin/env python3
"""
Raspberry Pi 5 Servo Controller ROS Node (gpiozero version)

This node subscribes to joint command topics and controls servos
via Raspberry Pi GPIO using gpiozero library.

Hardware:
- Raspberry Pi 5
- 3x MG996R servo motors (Joint 1, 2, 3)
- 1x Micro servo (Joint 5 - gripper)
- External 5-6V power supply (REQUIRED!)

GPIO Connections:
- GPIO 17 -> Joint_1
- GPIO 27 -> Joint_2
- GPIO 22 -> Joint_3
- GPIO 24 -> Joint_5 (Joint_4 not used in your 4-DOF robot)

Author: Team 13
Date: December 3, 2025
"""

import rospy
from gpiozero import AngularServo # type: ignore
from std_msgs.msg import Float64
import math

# --- CONFIGURATION FOR PI 5 ---
PIN_JOINT1 = 17  # Base
PIN_JOINT2 = 27  # Shoulder
PIN_JOINT3 = 22  # Elbow
PIN_JOINT5 = 24  # Gripper (Joint_4 not used)

# Pulse width settings (Standard for MG996R/SG90)
MIN_PULSE = 0.0005  # 0.5ms
MAX_PULSE = 0.0025  # 2.5ms

class PiServoController:
    def __init__(self):
        rospy.init_node('pi_servo_controller', anonymous=False)
        rospy.loginfo("Initializing Pi 5 Servo Controller (gpiozero)...")

        # Initialize Servos
        # We use min_angle=-90/max_angle=90 to map standard servo range
        self.joint1 = AngularServo(PIN_JOINT1, min_angle=-90, max_angle=90, 
                                    min_pulse_width=MIN_PULSE, max_pulse_width=MAX_PULSE)
        self.joint2 = AngularServo(PIN_JOINT2, min_angle=-90, max_angle=90, 
                                    min_pulse_width=MIN_PULSE, max_pulse_width=MAX_PULSE)
        self.joint3 = AngularServo(PIN_JOINT3, min_angle=-90, max_angle=90, 
                                    min_pulse_width=MIN_PULSE, max_pulse_width=MAX_PULSE)
        self.joint5 = AngularServo(PIN_JOINT5, min_angle=-90, max_angle=90, 
                                    min_pulse_width=MIN_PULSE, max_pulse_width=MAX_PULSE)

        # Initialize to 0 position (center)
        self.joint1.angle = 0
        self.joint2.angle = 0
        self.joint3.angle = 0
        self.joint5.angle = 0

        # --- SUBSCRIBERS ---
        # These topics MUST match what cubic_trajectory_planner.py publishes!
        # Your trajectory planner publishes to: /Joint_X/command
        self.sub_j1 = rospy.Subscriber('/Joint_1/command', Float64, self.cb_j1)
        self.sub_j2 = rospy.Subscriber('/Joint_2/command', Float64, self.cb_j2)
        self.sub_j3 = rospy.Subscriber('/Joint_3/command', Float64, self.cb_j3)
        self.sub_j5 = rospy.Subscriber('/Joint_5/command', Float64, self.cb_j5)
        # Note: Joint_4 not used in your 4-DOF robot

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("✅ Pi 5 Servo Controller Ready")
        rospy.loginfo(f"   GPIO pins: {PIN_JOINT1}, {PIN_JOINT2}, {PIN_JOINT3}, {PIN_JOINT5}")
        rospy.loginfo("   Subscribed to: /Joint_1/command, /Joint_2/command, /Joint_3/command, /Joint_5/command")
        rospy.loginfo("   Waiting for joint commands...")

    # --- CALLBACKS ---
    # Convert ROS radians to Servo Degrees
    def rad_to_deg(self, radians):
        """
        Convert radians to degrees and clamp to -90 to 90 range
        
        Args:
            radians: Angle in radians
            
        Returns:
            Angle in degrees, clamped to [-90, 90]
        """
        deg = math.degrees(radians)
        # Clamp to -90 to 90 range (servo range)
        return max(-90, min(90, deg))

    def cb_j1(self, msg):
        angle = self.rad_to_deg(msg.data)
        self.joint1.angle = angle
        rospy.logdebug(f"Joint 1: {msg.data:.3f} rad -> {angle:.1f}°")

    def cb_j2(self, msg):
        angle = self.rad_to_deg(msg.data)
        self.joint2.angle = angle
        rospy.logdebug(f"Joint 2: {msg.data:.3f} rad -> {angle:.1f}°")
    
    def cb_j3(self, msg):
        angle = self.rad_to_deg(msg.data)
        self.joint3.angle = angle
        rospy.logdebug(f"Joint 3: {msg.data:.3f} rad -> {angle:.1f}°")

    def cb_j5(self, msg):
        angle = self.rad_to_deg(msg.data)
        self.joint5.angle = angle
        rospy.logdebug(f"Joint 5: {msg.data:.3f} rad -> {angle:.1f}°")

    def shutdown(self):
        rospy.loginfo("Shutting down servos...")
        try:
            self.joint1.detach()
            self.joint2.detach()
            self.joint3.detach()
            self.joint5.detach()
        except:
            pass
        rospy.loginfo("✅ Servo controller shutdown complete")

if __name__ == '__main__':
    try:
        controller = PiServoController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()