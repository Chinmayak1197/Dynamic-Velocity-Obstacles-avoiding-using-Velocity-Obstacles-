**Motion Planning in Dynamic Environment**  

- Final code for Dynamic velocity obstacle for Motion Planning.  To run the code, please gitclone the repository.  
- To run the DWA planner on Gazebo, ros-launch baseline\_launch1.launch. Choose the husky robot on any of the inbuilt Turtle-Bot maps or other worlds.  
- To run the Python/Pygame code, download the VO/APF folder. Then run the python code on your preferred IDE. 

**Artificial Potential Field:** 

Robot moving in an hospital lobby. 
<br />
<a href="url"><img src="https://github.com/Chinmayak1197/Dynamic-Velocity-Obstacles-avoiding-using-Velocity-Obstacles-/blob/main/Videos%20and%20Results/APF_Python.gif" height="250" width="400" ></a>
<br />
**Limitation of Artificial Potential Field:** 

-  Stuck at Local Minima.

<a href="url"><img src="https://github.com/Chinmayak1197/Dynamic-Velocity-Obstacles-avoiding-using-Velocity-Obstacles-/blob/main/Videos%20and%20Results/APF_Local_Minima.gif" height="250" width="400" ></a>
<br />
**Generalized Velocity Obstacle:**  

Robot moving in the dynamic environment, where red dot represents the robot and the white dot represents the dynamic obstacles. 

<a href="url"><img src="https://github.com/Chinmayak1197/Dynamic-Velocity-Obstacles-avoiding-using-Velocity-Obstacles-/blob/main/Videos%20and%20Results/VO_2.gif" height="250" width="400" ></a>
<br />
**References** : 

- [https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/PotentialFi eldPlanning/potential_field_planning.py ](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/PotentialFieldPlanning/potential_field_planning.py) 
- [https://stackoverflow.com/questions/30015787/random-movement-pygame  ](https://stackoverflow.com/questions/30015787/random-movement-pygame)
- [https://github.com/RahulSajnani/Robotics-Planning-and-Navigation  ](https://github.com/RahulSajnani/Robotics-Planning-and-Navigation)
- <http://wiki.ros.org/motion_planning>[ http://wiki.ros.org/answers.ros.org  ](http://wiki.ros.org/answers.ros.org)
- [https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/ ](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) 
- <https://www.theconstructsim.com/>[ http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)
