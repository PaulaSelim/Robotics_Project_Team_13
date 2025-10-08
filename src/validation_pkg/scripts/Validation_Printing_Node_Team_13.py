#!/usr/bin/env python3

import rospy

def main():
    # Initialize the ROS node
    # The 'anonymous=True' flag ensures this node has a unique name
    rospy.init_node('validation_printer_node', anonymous=True)

    rospy.loginfo("Hello world")

    # rospy.spin() keeps the node from exiting until it's shut down
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass