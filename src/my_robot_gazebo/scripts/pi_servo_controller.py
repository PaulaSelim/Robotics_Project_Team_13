#!/usr/bin/env python3
"""
Raspberry Pi 5 Servo Controller ROS Node

This node subscribes to joint command topics and controls servos
via Raspberry Pi GPIO PWM pins.

Hardware:
- Raspberry Pi 5
- 3x MG996R servo motors (Joint 1, 2, 3)
- 1x Micro servo (Joint 5 - gripper)
- External 5-6V power supply (REQUIRED!)

GPIO Connections:
- GPIO 18 (Pin 12) -> Joint_1
- GPIO 13 (Pin 33) -> Joint_2
- GPIO 12 (Pin 32) -> Joint_3
- GPIO 19 (Pin 35) -> Joint_5

Author: Team 13
Date: December 3, 2025
"""

import rospy
from std_msgs.msg import Float64
import math

try:
    import pigpio # type: ignore
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False
    rospy.logwarn("pigpio not installed. Install with: sudo apt install python3-pigpio")


class PiServoController:
    """
    ROS node to control servos via Raspberry Pi GPIO
    """
    
    def __init__(self):
        rospy.init_node('pi_servo_controller', anonymous=False)
        
        # GPIO pins for servos (BCM numbering)
        self.PIN_JOINT1 = 18  # PWM0 channel
        self.PIN_JOINT2 = 13  # PWM1 channel
        self.PIN_JOINT3 = 12  # PWM0 channel
        self.PIN_JOINT5 = 19  # PWM1 channel
        
        # Servo parameters
        self.SERVO_MIN_PULSE = 500   # Minimum pulse width (microseconds)
        self.SERVO_MAX_PULSE = 2500  # Maximum pulse width (microseconds)
        self.SERVO_CENTER = 1500     # Center position
        
        # Servo limits (adjust based on physical constraints)
        self.SERVO_MIN_ANGLE = 0     # degrees
        self.SERVO_MAX_ANGLE = 180   # degrees
        
        # Current positions (0-180 degrees)
        self.current_pos = {
            'joint1': 90.0,
            'joint2': 90.0,
            'joint3': 90.0,
            'joint5': 90.0
        }
        
        # Initialize pigpio
        if not PIGPIO_AVAILABLE:
            rospy.logerr("Cannot start - pigpio not available!")
            rospy.signal_shutdown("pigpio not available")
            return
        
        try:
            self.pi = pigpio.pi() # type: ignore
            if not self.pi.connected:
                rospy.logerr("Failed to connect to pigpio daemon!")
                rospy.logerr("Start with: sudo pigpiod")
                rospy.signal_shutdown("pigpio daemon not running")
                return
        except Exception as e:
            rospy.logerr(f"Failed to initialize pigpio: {e}")
            rospy.signal_shutdown("pigpio initialization failed")
            return
        
        # Set servo mode on GPIO pins
        self.pi.set_mode(self.PIN_JOINT1, pigpio.OUTPUT) # type: ignore
        self.pi.set_mode(self.PIN_JOINT2, pigpio.OUTPUT) # type: ignore
        self.pi.set_mode(self.PIN_JOINT3, pigpio.OUTPUT) # type: ignore
        self.pi.set_mode(self.PIN_JOINT5, pigpio.OUTPUT) # type: ignore
        
        # Initialize servos to center position
        self.set_servo_pulse(self.PIN_JOINT1, self.SERVO_CENTER)
        self.set_servo_pulse(self.PIN_JOINT2, self.SERVO_CENTER)
        self.set_servo_pulse(self.PIN_JOINT3, self.SERVO_CENTER)
        self.set_servo_pulse(self.PIN_JOINT5, self.SERVO_CENTER)
        
        rospy.sleep(0.5)
        
        # Subscribe to joint command topics
        self.sub1 = rospy.Subscriber('/Joint_1/command', Float64, self.joint1_callback)
        self.sub2 = rospy.Subscriber('/Joint_2/command', Float64, self.joint2_callback)
        self.sub3 = rospy.Subscriber('/Joint_3/command', Float64, self.joint3_callback)
        self.sub5 = rospy.Subscriber('/Joint_5/command', Float64, self.joint5_callback)
        
        rospy.loginfo("✅ Raspberry Pi Servo Controller initialized")
        rospy.loginfo(f"   GPIO pins: {self.PIN_JOINT1}, {self.PIN_JOINT2}, {self.PIN_JOINT3}, {self.PIN_JOINT5}")
        rospy.loginfo("   Waiting for joint commands...")
    
    def set_servo_pulse(self, gpio_pin, pulse_width):
        """
        Set servo position using pulse width
        
        Args:
            gpio_pin: GPIO pin number (BCM)
            pulse_width: Pulse width in microseconds (500-2500)
        """
        try:
            self.pi.set_servo_pulsewidth(gpio_pin, pulse_width)
        except Exception as e:
            rospy.logerr(f"Error setting servo on pin {gpio_pin}: {e}")
    
    def angle_to_pulse(self, angle_degrees):
        """
        Convert angle (0-180 degrees) to pulse width (500-2500 us)
        
        Args:
            angle_degrees: Servo angle in degrees (0-180)
            
        Returns:
            Pulse width in microseconds
        """
        # Constrain angle
        angle_degrees = max(self.SERVO_MIN_ANGLE, min(self.SERVO_MAX_ANGLE, angle_degrees))
        
        # Map angle to pulse width
        pulse = self.SERVO_MIN_PULSE + (angle_degrees / 180.0) * (self.SERVO_MAX_PULSE - self.SERVO_MIN_PULSE)
        
        return int(pulse)
    
    def radians_to_servo_angle(self, radians):
        """
        Convert radians (from ROS) to servo angle (0-180 degrees)
        
        Args:
            radians: Joint angle in radians (-π to +π)
            
        Returns:
            Servo angle in degrees (0-180)
        """
        # Convert radians to degrees
        degrees = math.degrees(radians)
        
        # Shift from [-180, +180] to [0, 180] range
        servo_angle = degrees + 90.0
        
        # Constrain to valid range
        servo_angle = max(0, min(180, servo_angle))
        
        return servo_angle
    
    def joint1_callback(self, msg):
        """Callback for Joint 1 commands"""
        angle = self.radians_to_servo_angle(msg.data)
        pulse = self.angle_to_pulse(angle)
        self.set_servo_pulse(self.PIN_JOINT1, pulse)
        self.current_pos['joint1'] = angle
        rospy.logdebug(f"Joint 1: {msg.data:.3f} rad -> {angle:.1f}° -> {pulse} us")
    
    def joint2_callback(self, msg):
        """Callback for Joint 2 commands"""
        angle = self.radians_to_servo_angle(msg.data)
        pulse = self.angle_to_pulse(angle)
        self.set_servo_pulse(self.PIN_JOINT2, pulse)
        self.current_pos['joint2'] = angle
        rospy.logdebug(f"Joint 2: {msg.data:.3f} rad -> {angle:.1f}° -> {pulse} us")
    
    def joint3_callback(self, msg):
        """Callback for Joint 3 commands"""
        angle = self.radians_to_servo_angle(msg.data)
        pulse = self.angle_to_pulse(angle)
        self.set_servo_pulse(self.PIN_JOINT3, pulse)
        self.current_pos['joint3'] = angle
        rospy.logdebug(f"Joint 3: {msg.data:.3f} rad -> {angle:.1f}° -> {pulse} us")
    
    def joint5_callback(self, msg):
        """Callback for Joint 5 (gripper) commands"""
        angle = self.radians_to_servo_angle(msg.data)
        pulse = self.angle_to_pulse(angle)
        self.set_servo_pulse(self.PIN_JOINT5, pulse)
        self.current_pos['joint5'] = angle
        rospy.logdebug(f"Joint 5: {msg.data:.3f} rad -> {angle:.1f}° -> {pulse} us")
    
    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("Shutting down servo controller...")
        
        # Stop all servos
        self.pi.set_servo_pulsewidth(self.PIN_JOINT1, 0)
        self.pi.set_servo_pulsewidth(self.PIN_JOINT2, 0)
        self.pi.set_servo_pulsewidth(self.PIN_JOINT3, 0)
        self.pi.set_servo_pulsewidth(self.PIN_JOINT5, 0)
        
        # Disconnect
        self.pi.stop()
        rospy.loginfo("✅ Servo controller shutdown complete")


def main():
    try:
        controller = PiServoController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
