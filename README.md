# iiwa
ros/rosjava packages for running the kuka iiwa.

# Getting started
This repository contains two packages.
* 'iiwa' contains the message and c++ code to send commands to the robot.
* 'iiwa_java_interface' contains the java code that is run on the robot side and listens for commands.

Running 'catkin_make' should build both. Alternatively, we also include the jar
files in 'iiwa_java_interface/JavaNode' so that they may be copied directly.

You should begin by adding 'iiwa_java_inteface/JavaNode/ROScom.java' to your
Sunrise Workbench. Then copy the jar files into the Workbench and add them as
libraries.

You may now need to edit 'ROScom.java'. Set the variables 'my_ip' and
'my_master' with the necessary values for your robot. In addition, if your
Sunrise Workbench is version >= 1.5, you should uncomment the corresponding
line in the 'run' method.

At this point, running 'ROScom' on the iiwa should connect it to the master
node. If this works you can begin writing your controller. A template for this
is given in the 'iiwa' package.
