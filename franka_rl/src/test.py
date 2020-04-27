
import rospy
import numpy as np
import rospkg
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from collections import defaultdict
import matplotlib.pyplot as plt
if __name__ == '__main__':
    rospy.init_node('training', anonymous=True, log_level=rospy.WARN)
    k = np.arange(250)
    result = np.ones(250)
    plt.figure(figsize = (8,6))
    plt.plot(k, result)
    plt.show()
