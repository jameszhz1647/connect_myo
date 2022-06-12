# connect_myo

**This is a ROS package to connect 4 Myos by launching 2 separate Ros nodes:**

1. In the first terminal: `roslaunch connect_myo myo.launch myo_arm:=left`
2. in the second terminal: `roslaunch connect_myo myo.launch myo_arm:=right`

The EMG data are published at Ros topics:

​	`/left_upper_myo/myo_emg`

​	`	/left_lower_myo/myo_emg`

​	`	/right_lower_myo/myo_emg`

​	`/right_upper_myo/myo_emg`

The IMU data are published at Ros topics:

​	`/left_upper_myo/myo_imu`

​	`/left_lower_myo/myo_imu`

​	`/right_lower_myo/myo_imu`

​	`/right_upper_myo/myo_imu`



