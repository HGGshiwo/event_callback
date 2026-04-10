import time

import pytest
import rospy
from std_msgs.msg import String


@pytest.mark.ros
def test_ros(test_env):
    """验证 ros 组件"""
    ros_topic = test_env["ros_topic"]
    manager = test_env["manager"]
    # time.sleep(1)
    pub = rospy.Publisher(ros_topic, String, queue_size=1)
    for i in range(100):
        if pub.get_num_connections() != 0:
            break
        time.sleep(0.1)
    else:
        assert 0 == 1, "timeout!"
    pub.publish(String("Hello"))
    time.sleep(0.1)
    assert manager.stats["ros_msg_count"] == 1
