import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/catscanners/Documents/find-my-kitten/ros2_ws/install/py_pubsub'
