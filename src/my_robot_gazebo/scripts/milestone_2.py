#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import Float64     

rospy.init_node('milestone_2_controller', anonymous=False)  



pub1 = rospy.Publisher('/Joint_1/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/Joint_2/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/Joint_3/command', Float64, queue_size=10)
pub4 = rospy.Publisher('/Joint_4/command', Float64, queue_size=10)
pub5 = rospy.Publisher('/Joint_5/command', Float64, queue_size=10)
pub6 = rospy.Publisher('/milestone_2_pose', PoseStamped, queue_size=10)


joint_1 = Float64()
joint_2 = Float64()
joint_3 = Float64()
joint_4 = Float64()
joint_5 = Float64()

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

while 1 and not rospy.is_shutdown():
   
    if flag == 1:
        
        joint_1 = Float64(90 * (math.pi / 180))
        joint_2 = Float64(90 * (math.pi / 180))
        joint_3 = Float64(90 * (math.pi / 180))
        joint_4 = Float64(0 * (math.pi / 180)) #this servo will not be used
        joint_5 = Float64(90 * (math.pi / 180))

      
        pub1.publish(joint_1)
        pub2.publish(joint_2)
        pub3.publish(joint_3)
        pub4.publish(joint_4)
        pub5.publish(joint_5)

        # Publishing the PoseStamped message to pub6
        pub6.publish(end_eff)
        flag = 0
    rate.sleep()

