#example procedures for simulator:

1. start up simulator

		roslaunch cwru_baxter_sim baxter_world.launch

2. wait for the robot (or Gazebo) to finish coming up; then enable the robot with:

		rosrun baxter_tools enable_robot.py -e

3. start up the action servers and transform publishers with the following commands in separate terminals

		rosrun baxter_traj_streamer  traj_interpolator_as

		rosrun baxter_cartesian_moves baxter_cart_move_as

		roslaunch cwru_baxter_launch yale_gripper_xform.launch         # to see gripper frame in rviz

4. Watch out for the following; kinect transform is different for Gazebo vs real Baxter
		
		roslaunch cwru_baxter_sim kinect_xform.launch

5.  start up rviz and the example sensor-guided motion node
		
		rosrun rviz rviz (and set display to see kinect/depth/points and gripper frame)

	4. start test program

		rosrun merry_motionplanner merry_motionplanner_main

* OR: use the handy launch file.  After starting the robot, run the following launch file:

		roslaunch merry_motionplanner  merry_motionplanner.launch


==================================================================================

#example procedures for real robot:

	1. start baxter	

		baxter_master # set the remote roscore master in each terminal

		rosrun baxter_tools enable_robot.py -e

	2. start the Kinect driver

		baxter_master

		roslaunch cwru_baxter_launch kinect.launch

	3. start up the action servers and transform publishers

		baxter_master

		rosrun baxter_traj_streamer  traj_interpolator_as

		&

		baxter_master

		rosrun baxter_cartesian_moves baxter_cart_move_as

	3. start rviz  (and set display to see kinect/depth/points and gripper frame)

		baxter_master

		rosrun rviz rviz

	4. start test program

		baxter_master

		rosrun merry_motionplanner merry_motionplanner_main










