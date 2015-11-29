example procedures:

	1. start baxter	

		baxter_master # set the remote roscore master in each terminal

		rosrun baxter_tools enable_robot.py -e

	2. start the Kinect driver

		baxter_master

		roslaunch cwru_baxter_launch kinect.launch

	3. start rviz

		baxter_master

		rosrun rviz rviz

	4. start the ROS motor driver

		baxter_master

		rosrun baxter_gripper dynamixel_motor_node

	5. command motor angles

		baxter_master

		rosrun merry_gripper merry_gripper_main

		or:

		rosrun baxter_gripper baxter_gripper_test

		or:

		rostopic pub dynamixel_motor1_cmd std_msgs/Int16 3600  # Under current conditions, the useful range of angle commands is approximately 3000 to 4000. release to grasp
