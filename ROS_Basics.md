# ROS Beginner Tutorial
## Chapter 1

*Here I try to collect the useful ROS commands from the course I am taking right now*

---
1. roslaunch is the command used to launch a ROS program. 
```
roslaunch <package_name> <launch_file>
```
2. To go directly to a package:
```
roscd <package_name>
```
3. Launch files have nodes and each node contains:
  * pkg="package_name" # Name of the package that contains the code of the ROS program to execute
  * type="python_file_name.py" # Name of the program file that we want to execute
  * name="node_name" # Name of the ROS node that will launch our Python file
  * output="type_of_output" # Through which channel you will print the output of the Python file
4. To locate an specific package
```
rospack list | grep my_package
```
5. Sometimes ROS won't detect a new package when you have just created it, so you won't be able to do a roslaunch. In this case, you can force ROS to do a refresh of its package list with the command:
```
rospack profile
```
6. Add execution permission to your python script
```
chmod +x name_of_file.py
```
7. OS nodes are basically programs made in ROS. The ROS command to see what nodes are actually running in a computer is:
```
rosnode list 
```
8. Get info about a node:
```
rosnode info /rosnode_name
```
9. In case you only need to compile one package
```
catkin_make --only-pkg-with-deps <package_name
```
---
10. Parameter Server:A Parameter Server is a dictionary that ROS uses to store parameters. These parameters can be used by nodes at runtime and are normally used for static data, such as configuration parameters.
```
rosparam list 
```
11. Get value of specific parameter:
```
rosparam get <parameter_name>
```
12. Set a parameter:
```
rosparam set <parameter_name> <value>
```
13. ROSCORE: In order to have all of this working, we need to have a roscore running. The roscore is the main process that manages all of the ROS system. You always need to have a roscore running in order to work with ROS. The command that launches a roscore is:
```
roscore 
```
 

  
