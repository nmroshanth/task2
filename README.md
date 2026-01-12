# task2
First run the model by  ros2 launch arm_description ros2_control.launch.py /n
in another terminal run ros2 run controller_manager spawner left_arm_position_controller /n
ros2 run controller_manager spawner joint_state_broadcaster /n
ros2 run controller_manager spawner joint_state_broadcaster /n
ros2 run dual_arm_control cartesian_jacobian_position_control /n
