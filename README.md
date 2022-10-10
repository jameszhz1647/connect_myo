# connect_myo

**This is a ROS package to connect 4 Myos by launching 2 separate Ros nodes:**

1. Plug the first dongle, type `roslaunch connect_myo left_myos.launch`in the first terminal.
2. Plug the second dongle, type `roslaunch connect_myo right_myos.launch`in the second terminal.

**If myos are not paired with their topics, just reset the corresponding rosparams in the terminal:**

For left two myos: `rosparam set /left_myo_name "['left_lower_myo', 'left_upper_myo']"` 

For right two myos:  `rosparam set /right_myo_name "['right_lower_myo', 'right_upper_myo']"` 

**The EMG data are published at Ros topics:**

​	`/left_upper_myo/myo_emg`

​	`	/left_lower_myo/myo_emg`

​	`	/right_lower_myo/myo_emg`

​	`/right_upper_myo/myo_emg`

**The IMU data are published at Ros topics:**

​	`/left_upper_myo/myo_imu`

​	`/left_lower_myo/myo_imu`

​	`/right_lower_myo/myo_imu`

​	`/right_upper_myo/myo_imu`



## Record EMG/IMU

**left arm:**

`rosbag record -O [xxx.bag] /left_lower_myo/myo_emg /left_lower_myo/myo_imu /left_upper_myo/myo_emg /left_upper_myo/myo_imu`

**right arm:**

`rosbag record -O [xxx.bag] /right_lower_myo/myo_emg /right_lower_myo/myo_imu /right_upper_myo/myo_emg /right_upper_myo/myo_imu`

**Convert bag to csv file:**

`rosrun rosbag_to_csv rosbag_to_csv.py`

