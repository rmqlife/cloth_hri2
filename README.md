### scripts

launch_ros_pkg.sh,  launch all the dependency of the ros, like YUMI, real sense

record_rosbag.sh,  record the topics of rosbag, use to collect training data

### purpose

scripts/init.py, initialize the end-effectors to a fixed position from a npz file

scripts/talker.py, online testing

scripts/reader.py, record the data into a npz file for training the model