1. 
	
#	roslaunch cwru_baxter_sim baxter_world.launch

or

#	baxter_master

2. 

#	rosrun baxter_tools enable_robot.py -e

3. 

#	roslaunch cwru_baxter_sim kinect_xform.launch

or

#	roslaunch cwru_baxter_launch kinect.launch

4. 

#	rosrun baxter_traj_streamer traj_interpolator_as

5. 

#	rosrun baxter_cartesian_moves baxter_cart_move_as

6.

#	rosrun rviz rviz

7.	

#	rosrun merry_coordinator merry_coordinator


