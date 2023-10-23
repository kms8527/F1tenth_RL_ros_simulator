# F1tenth_RL_ros_simulator

this simulator is made for my Reinforcement learning Project.
this code can spawn multiple vehicles.

1'th vehicle drive topic : drive topic name + "0"

2'th vehicle drive topic : drive topic name + "1"

3'th vehicle drive topic : drive topic name + "2"

...

1'th vehicle odom topic : odom topic name + "0"

2'th vehicle odom topic : odom topic name + "1"

3'th vehicle odom topic : odom topic name + "2"

...


you can set initial pose of 1'th vehicle with "2D Pose Estimate"

you can set initial pose of 2'th vehicle with "2D Nav Goal"


if collision occurs,it can restart.
but obstacles is not detected in 2D lidar, it recevices data directly using odom topic



https://github.com/kms8527/F1tenth_RL_ros_simulator/assets/44738230/f87b52d7-4ed2-44cb-9337-e8501fed0437




