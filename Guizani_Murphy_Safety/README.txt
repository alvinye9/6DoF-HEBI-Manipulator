#############continuity#################
####CDT Guizani Skander, CDT Alexander Murphy
#### For anyone questions please contact via email: iskanderguizani1@gmail.com, alexmurph16@gmail.com

To run the demo on ROS Noetic, Ubuntu LTS 20.04. For Hardware, you need your computer runnig linux and Realsense Depth 
camera D455 (in my case) 
*) To open a terminal in linux click: ctrl + alt + t (at the same time) 
*) you should be in the main directory but to move to the catkin workspace, type $cd catkin_ws 
*) type in in the termina: $ls , to see the files inside of your directory. 
*) First you need to build and source your catkin workspace. 
*) Make a package in your catkin workspace, you can follow this tutorial to make a package. 
http://wiki.ros.org/ROS/Tutorials/CreatingPackage  and then put the python files in the same folder inside the src file.
*) Open Terminal, go to catkin workspace and type: $source devel/setup.bash$
*) Put the file #Camera.py, #final_robot.py, #realsense.py in the same path. 
*) if you download these files to a new computer with a new ros environment
*) to make these files executable: go to directory where you put all the python files inside your package inside your catkin workspace. 
*) then for each file, type in: $chmod +x /name of the file
*) in a terminal: type $rosrun /name_of_your_package final_robot.py$, in this my case it's $rosrun skander_final final_robot.py
*) if you get an error saying that these files are not executable you need to convert the files from the DOS format to the Unix format 
*) type $dos2unix /name_of_the_file
*) Make sure you have python 3 and std_msgs.msgs installed in your ros. 
*) Other packages you need: # numpy, math, scipy.spatial, rospy, pyrealsense2, std_msgs.msg, imutilis, decimal, time.








 For the torque Safety demo : 
*) Open the Safety_Combined in Downloads under the Coding Folder
*) Run Jacobian_Inverse_Final.m and JAN7_final_robot_COM_functions.m prior to running code only once: Already run on computer shouldn't be necessary
*) Add hebi and symbolic_function to path
*) insert rosinit into the command window and hit enter to initialize the ROS network
*) Hit Run
*) Hold A to intitialize robot setup and press A to rerun safety demo
*) Press B to run touch demo only

For controlling Robot:
*) Upper Left Joystick: left to right rotates robot at base
*) Upper Left Joystick: up and down rotates the first link of the robot
*) Lower Left Joystick: up and down rotates the second link of the robot

===============> These demos run together, without the points sent from matlab you cant see anything. 
===============> Also to see if you're publishing something in matlab in a new terminal: $rostopic list
===============> To see what are you exactly publishing: $rostopic echo /name_of_the_publisher