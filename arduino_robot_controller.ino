/*
 * 4-DOF Robotic Arm Controller
 * Controls 3x MG996R servos and 1x Micro servo
 * Receives joint angles via Serial from ROS trajectory planner
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
 * Serial Protocol:
 * Format: "J1:angle1,J2:angle2,J3:angle3,J5:angle5\n"
 * Example: "J1:45.5,J2:-30.2,J3:60.0,J5:90.0\n"
 * Angles in DEGREES (-180 to +180)
 */

#include <Servo.h>

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

// Serial communication
String inputBuffer = "";
unsigned long lastUpdate = 0;

// LED for status indication
const int LED_PIN = LED_BUILTIN;
bool ledState = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);  // High baud rate for responsive control
  
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
  
  // Startup message
  delay(1000);
  Serial.println("===========================================");
  Serial.println("4-DOF Robot Arm Controller Ready");
  Serial.println("===========================================");
  Serial.println("Waiting for commands...");
  Serial.println("Format: J1:angle1,J2:angle2,J3:angle3,J5:angle5");
  Serial.println("===========================================");
  
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Read serial commands
  readSerialCommands();
  
  // Smooth movement update
  if (millis() - lastUpdate >= UPDATE_INTERVAL) {
    updateServoPositions();
    lastUpdate = millis();
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

void processCommand(String cmd) {
  // Blink LED to show command received
  digitalWrite(LED_PIN, HIGH);
  
  // Parse command: "J1:45.5,J2:-30.2,J3:60.0,J5:90.0"
  cmd.trim();
  
  // Split by comma
  int idx1 = cmd.indexOf(',');
  int idx2 = cmd.indexOf(',', idx1 + 1);
  int idx3 = cmd.indexOf(',', idx2 + 1);
  
  if (idx1 == -1 || idx2 == -1 || idx3 == -1) {
    Serial.println("ERROR: Invalid command format");
    digitalWrite(LED_PIN, LOW);
    return;
  }
  
  // Extract joint angles
  String j1_str = cmd.substring(0, idx1);
  String j2_str = cmd.substring(idx1 + 1, idx2);
  String j3_str = cmd.substring(idx2 + 1, idx3);
  String j5_str = cmd.substring(idx3 + 1);
  
  // Parse angles (format: "J1:45.5")
  float j1 = parseJointAngle(j1_str);
  float j2 = parseJointAngle(j2_str);
  float j3 = parseJointAngle(j3_str);
  float j5 = parseJointAngle(j5_str);
  
  // Convert from [-180, +180] to [0, 180] servo range
  target_pos[0] = constrainAngle(j1 + 90.0, 0);
  target_pos[1] = constrainAngle(j2 + 90.0, 1);
  target_pos[2] = constrainAngle(j3 + 90.0, 2);
  target_pos[3] = constrainAngle(j5 + 90.0, 3);
  
  // Debug output
  Serial.print("Target: J1=");
  Serial.print(target_pos[0]);
  Serial.print(" J2=");
  Serial.print(target_pos[1]);
  Serial.print(" J3=");
  Serial.print(target_pos[2]);
  Serial.print(" J5=");
  Serial.println(target_pos[3]);
  
  digitalWrite(LED_PIN, LOW);
}

float parseJointAngle(String joint_str) {
  // Format: "J1:45.5" -> extract 45.5
  int colonIdx = joint_str.indexOf(':');
  if (colonIdx == -1) return 0.0;
  
  String angle_str = joint_str.substring(colonIdx + 1);
  return angle_str.toFloat();
}

float constrainAngle(float angle, int joint_idx) {
  // Constrain angle to valid servo range
  return constrain(angle, SERVO_MIN[joint_idx], SERVO_MAX[joint_idx]);
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
