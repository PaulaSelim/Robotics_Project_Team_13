/*
 * 4-DOF Robotic Arm ROS Controller
 * Controls 3x MG996R servos and 1x Micro servo via ROS
 * Subscribes to joint command topics from ROS trajectory planner
 * 
 * Hardware:
 * - Arduino Uno/Mega
 * - 3x MG996R servo motors (Joint 1, 2, 3)
 * - 1x Micro servo (Joint 5 - gripper)
 * - External 5-6V power supply for servos (REQUIRED!)
 * 
 * Connections:
 * Joint_1 (MG996R) -> Pin 9
 * Joint_2 (MG996R) -> Pin 10
 * Joint_3 (MG996R) -> Pin 11
 * Joint_5 (Micro)  -> Pin 6
 * 
 * ROS Setup:
 * 1. Install rosserial: sudo apt install ros-noetic-rosserial-arduino
 * 2. Install Arduino library: 
 *    cd ~/Arduino/libraries
 *    rosrun rosserial_arduino make_libraries.py .
 * 3. Upload this sketch to Arduino
 * 4. Run: rosrun rosserial_python serial_node.py /dev/ttyACM1
 */

#include <ros.h>
#include <std_msgs/Float64.h>
#include <Servo.h>

// ROS node handle
ros::NodeHandle nh;

// Servo objects
Servo servo1;  // Joint_1 - Base rotation (MG996R)
Servo servo2;  // Joint_2 - Shoulder (MG996R)
Servo servo3;  // Joint_3 - Elbow (MG996R)
Servo servo5;  // Joint_5 - Gripper (Micro servo)

// Servo pins
const int PIN_SERVO1 = 9;
const int PIN_SERVO2 = 10;
const int PIN_SERVO3 = 11;
const int PIN_SERVO5 = 6;

// Current servo positions (degrees, 0-180 range for servo.write())
float current_pos[4] = {90, 90, 90, 90};  // Start at neutral
float target_pos[4] = {90, 90, 90, 90};

// Servo limits (adjust based on your physical constraints)
const float SERVO_MIN[4] = {0, 0, 0, 0};
const float SERVO_MAX[4] = {180, 180, 180, 180};

// Movement parameters
const float MOVE_SPEED = 2.0;  // degrees per update cycle
const int UPDATE_INTERVAL = 20;  // milliseconds

unsigned long lastUpdate = 0;

// LED for status indication
const int LED_PIN = LED_BUILTIN;
bool ledState = false;

// Callback functions for each joint
void joint1_cb(const std_msgs::Float64& msg) {
  // Convert from radians to degrees, then to [0, 180] range
  float angle_deg = msg.data * 180.0 / PI;
  target_pos[0] = constrain(angle_deg + 90.0, SERVO_MIN[0], SERVO_MAX[0]);
  digitalWrite(LED_PIN, HIGH);
}

void joint2_cb(const std_msgs::Float64& msg) {
  float angle_deg = msg.data * 180.0 / PI;
  target_pos[1] = constrain(angle_deg + 90.0, SERVO_MIN[1], SERVO_MAX[1]);
  digitalWrite(LED_PIN, HIGH);
}

void joint3_cb(const std_msgs::Float64& msg) {
  float angle_deg = msg.data * 180.0 / PI;
  target_pos[2] = constrain(angle_deg + 90.0, SERVO_MIN[2], SERVO_MAX[2]);
  digitalWrite(LED_PIN, HIGH);
}

void joint5_cb(const std_msgs::Float64& msg) {
  float angle_deg = msg.data * 180.0 / PI;
  target_pos[3] = constrain(angle_deg + 90.0, SERVO_MIN[3], SERVO_MAX[3]);
  digitalWrite(LED_PIN, HIGH);
}

// ROS subscribers for each joint
ros::Subscriber<std_msgs::Float64> sub1("/Joint_1/command", &joint1_cb);
ros::Subscriber<std_msgs::Float64> sub2("/Joint_2/command", &joint2_cb);
ros::Subscriber<std_msgs::Float64> sub3("/Joint_3/command", &joint3_cb);
ros::Subscriber<std_msgs::Float64> sub5("/Joint_5/command", &joint5_cb);

void setup() {
  // Initialize ROS node
  nh.initNode();
  
  // Subscribe to joint command topics
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub5);
  
  // Attach servos to pins
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);
  servo5.attach(PIN_SERVO5);
  
  // Move to initial position (neutral)
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo5.write(90);
  
  // Setup LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Process ROS callbacks
  nh.spinOnce();
  
  // Smooth movement update
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    updateServoPositions();
    lastUpdate = millis();
  }
  
  delay(1);
}

void updateServoPositions() {
  // Smooth movement towards target positions
  bool moving = false;
  
  for (int i = 0; i < 4; i++) {
    float diff = target_pos[i] - current_pos[i];
    
    if (abs(diff) > 0.5) {  // Threshold for "close enough"
      moving = true;
      
      // Move towards target
      if (diff > 0) {
        current_pos[i] += min(MOVE_SPEED, diff);
      } else {
        current_pos[i] += max(-MOVE_SPEED, diff);
      }
    } else {
      current_pos[i] = target_pos[i];
    }
  }
  
  // Write to servos
  servo1.write((int)current_pos[0]);
  servo2.write((int)current_pos[1]);
  servo3.write((int)current_pos[2]);
  servo5.write((int)current_pos[3]);
  
  // Indicate movement with LED
  if (moving) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}