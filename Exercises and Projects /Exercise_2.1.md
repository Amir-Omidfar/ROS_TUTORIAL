# Exercise 2.1 

### Python_Script :
```
#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
var = Twist()
var.linear.x = 0.5
var.angular.z = 0.5
rate = rospy.Rate(2)
count = 0

while count != 20:
    pub.publish(var)
    count += 1
    rate.sleep()
var.linear.x = 0
var.angular.z = 0

while not rospy.is_shutdown():
    pub.publish(var)
    rate.sleep()

```
### Launch File :
```
<launch>
    <!-- My Package launch file -->
    <node pkg="exc2_1" type="move_robot.py" name="move_robot_node"  output="screen">
    </node>
</launch>
```