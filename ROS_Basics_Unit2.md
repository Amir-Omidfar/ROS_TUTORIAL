# ROS Topics

A topic is like a pipe. **Nodes use topics to publish information for other nodes so that they can communicate**. 
You can find out, at any time, the number of topics in the system by doing a rostopic list. You can also check for a specific topic.
Try below commands:
 ```
rostopic list 
rostopic info topic_name
rostopic list 
rostopic echo topic_name
rostopic echo topic_name -nl
```
Consider the code below:
```
#! /usr/bin/env python

import rospy                               # Import the Python library for ROS
from std_msgs.msg import Int32             # Import the Int32 message from the std_msgs package

rospy.init_node('topic_publisher')         # Initiate a Node named 'topic_publisher'
pub = rospy.Publisher('counter', Int32)    # Create a Publisher object, that will publish on the /counter topic
                                           #  messages of type Int32

rate = rospy.Rate(2)                       # Set a publish rate of 2 Hz
count = Int32()                            # Create a var of type Int32
count.data = 0                             # Initialize 'count' variable

while not rospy.is_shutdown():             # Create a loop that will go until someone stops the program execution
  pub.publish(count)                       # Publish the message within the 'count' variable
  count.data += 1                          # Increment 'count' variable
  rate.sleep() 
```
Points: 
1. **A publisher is a node that keeps publishing a message into a topic.**
2. **A topic is a channel that acts as a pipe, where other ROS nodes can either publish or read information.**
---
## Messages 
