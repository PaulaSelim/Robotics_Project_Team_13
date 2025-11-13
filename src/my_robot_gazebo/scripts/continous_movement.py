#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import PoseStamped
import numpy as np 
from std_msgs.msg import Float64     

rospy.init_node('milestone_2_controller', anonymous=False)  


pub0 = rospy.Publisher('/Joint_0/command', Float64, queue_size=10)
pub1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/Joint_4/command', Float64, queue_size=10)
pub5 = rospy.Publisher('/milestone_2_pose', PoseStamped, queue_size=10)

joint_0 = Float64()
joint_1 = Float64()
joint_2 = Float64()
joint_3 = Float64()
joint_4 = Float64()

end_eff = PoseStamped()
flag = 0
x=0.0
y=0.0
z=0.0

rate = rospy.Rate(10) # 10 Hz

def callback(data):
    global sub1
    global flag
    global x
    global y
    global z
    global end_eff

    if flag ==0:
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        print("X: ", x)
        print("Y: ", y)
        print("Z: ", z)
        end_eff = data
        flag = 1
        

sub1 = rospy.Subscriber('/end_effector_pose', PoseStamped, callback)

# Define a sequence of joint positions for continuous movement
demo_positions = [
    [0, 0.5, 0.5, 0.5, 1],
    [0.2, 0.4, 0.6, 0.8, 1],
    [-0.2, 0.3, 0.7, 0.9, 0.5],
    [0, 0, 0, 0, 0]
]

# Index to track the current position in the sequence
current_position_index = 0

while not rospy.is_shutdown():
    # Get the current joint positions from the sequence
    joint_positions = demo_positions[current_position_index]

    # Publish the joint positions
    pub0.publish(Float64(joint_positions[0]))
    pub1.publish(Float64(joint_positions[1]))
    pub2.publish(Float64(joint_positions[2]))
    pub3.publish(Float64(joint_positions[3]))
    pub4.publish(Float64(joint_positions[4]))

    # Move to the next position in the sequence
    current_position_index = (current_position_index + 1) % len(demo_positions)

    # Sleep to maintain the loop rate
    rate.sleep()

