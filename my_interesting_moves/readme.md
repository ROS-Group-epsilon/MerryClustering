# example procedures

1. start baxter simulator

	roslaunch cwru_baxter_sim baxter_world.launch

2. enable the robot

	rosrun baxter_tools enable_robot.py -e

3. server and client

	rosrun my_interesting_moves peng_interpolator_as

	rosrun my_interesting_moves peng_action_client_pre_pose

4. if you wanna see it in rviz:

	roslaunch cwru_baxter_sim  kinect_xform.launch

	rosrun rviz rviz
