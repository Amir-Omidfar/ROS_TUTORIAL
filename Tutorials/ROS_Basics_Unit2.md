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
Method of handling information in topics 
**Commands**
```
rosmsg show msg_name

```
---
## Subscribers 

So far writing to a topic has been done using **publishers**.<br />
**Subscriber** is a node that can read from a topic.
<br />
This is how to publish a message in a topic:
```
rostopic pub <topic_name> <message_type> <value>
```
If a message is published then you can read off it by subcscribing using :
```
rostopic echo <topic_name>
```
---
### Create Custom Messages
1. Create a directory named 'msg' inside your package
2. Inside this directory, create a file named Name_of_your_message.msg 
    * example Age.msg (in your msg file insert the msg format)
    ```
    float32 years
    float32 months
    float32 days
    ```
4. Modify CMakeLists.txt file 
    * Edit find_package()
      * find_package(catkin REQUIRED COMPONENTS
      rospy
      std_msgs
      message_generation)
    * Edit add_message_files()
      * add_message_files(
      FILES
      Age.msg
    ) 
    * Edit generate_messages()
      * generate_messages(
      DEPENDENCIES
      std_msgs
) 
    * Edit catkin_package()
6. Modify package.xml file 
      * Two additional lines:
      ```
      <build_depend>message_generation</build_depend>
      <build_export_depend>message_runtime</build_export_depend>
      <exec_depend>message_runtime</exec_depend>
      ```
8. Compile
    ```
    catkin_make
    source devel/setup.bash
    ```
    * VERY IMPORTANT: When you compile new messages, there is still an extra step before you can use the messages. You have to type in the Webshell, in the catkin_ws directory, the following command: source devel/setup.bash. 
    * This executes this bash file that sets, among other things, the newly generated messages created through the catkin_make.
    * If you don't do this, it might give you a python import error, saying it doesn't find the message generated.
    * To verify your message compilation type in:
    ```
    rosmsg info Age
    ```
10. Use in code


