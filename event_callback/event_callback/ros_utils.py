import logging
from typing import Any, Callable, Optional, Type

try:
    import rospy
    from rosgraph.masterapi import Master
except:
    rospy = None
    Master = None


def rospy_is_shutdown():
    return rospy is not None and rospy.is_shutdown()


def rospy_init_node(node_name: str, anonymous: bool = False):
    """初始化ros node, 避免ros设置logger"""
    if rospy.core.is_initialized():
        return
    _logger = logging.getLogger()
    _handler = [h for h in _logger.handlers]
    _level = _logger.level
    rospy.init_node(node_name, anonymous=anonymous)
    logger = logging.getLogger()
    logger.setLevel(_level)
    for handler in _handler:
        logger.addHandler(handler)


class rostopic_field:
    """自动收集topic对应的数据"""

    def __init__(
        self,
        topic_name: str,
        topic_type: Type,
        timeout: Optional[float] = 0.01,
        format: Optional[Callable] = None,
    ):
        """
        Args:
            topic_name (str): ros topic名称
            topic_type (Type): ros topic 类型
            timeout (float, optional): 消息超时时间，访问超时的消息返回None，单位seconds. Defaults to 0.01.
            format (Callable, optional): 格式化函数，对获取的msg进行自定义格式化
        """
        self._topic_name = topic_name
        self._topic_type = topic_type
        self._timeout = timeout
        self._msg = None
        self._format = format
        self._check_topic_type(topic_name, topic_type)
        # 保存订阅者引用，避免被GC回收导致订阅失效
        self._subscriber = rospy.Subscriber(topic_name, topic_type, self._msg_callback)

    def _get_topic_type(self, topic_name: str):
        """
        查询指定话题的类型
        :param topic_name: 话题名称（如 "/chatter"）
        :return: 话题类型字符串（如 "std_msgs/String"），若话题不存在返回None
        """
        try:

            master = Master(rospy.get_name())
            # 获取话题的所有连接信息
            topic_info = master.getTopicTypes()
            # 遍历查找目标话题
            for name, type_str in topic_info:
                if name == topic_name:
                    return type_str
            return None
        except Exception as e:
            rospy.logerr(f"查询话题类型失败: {e}")
            return None

    def _check_topic_type(self, topic_name: str, expected_type: Any):
        """
        带类型检查的订阅函数
        :param topic_name: 要订阅的话题名
        :param expected_type: 期望的消息类型类（如 String）
        """
        # 步骤1：获取期望类型的字符串表示（如 std_msgs/String）
        expected_type_str = expected_type._type

        # 步骤2：查询话题真实类型
        real_type_str = self._get_topic_type(topic_name)

        # 步骤3：类型检查与警告
        if real_type_str is None:
            # rospy.logwarn(f"警告：话题 {topic_name} 未找到，可能尚未发布！")
            return
        elif real_type_str != expected_type_str:
            raise ValueError(
                f"topic_name type dismatch! expected: {expected_type_str}, got: {real_type_str}"
            )

    def _msg_callback(self, msg: Any):
        """话题回调函数，更新最新消息"""
        self._msg = msg

    def _check_timeout(self) -> bool:
        """检查消息是否超时，超时返回True，未超时返回False"""
        if self._timeout is None:
            return False
        if not hasattr(self._msg, "header"):
            return False
        # 计算当前时间与消息时间的差值（秒）
        time_diff = (rospy.Time.now() - self._msg.header.stamp).to_sec()
        # 差值 >= 超时时间 → 超时
        return time_diff >= self._timeout

    @property
    def value(self):
        """获取处理后的话题值（含超时检查+格式化）"""
        if self._msg is None:
            return None
        if self._check_timeout():
            return None
        # 应用格式化函数（如果有）
        if self._format is not None:
            return self._format(self._msg)
        return self._msg


class rosparam_field:  # 修正拼写错误：filed → field
    """ROS参数操作类（get/set）"""

    def __init__(self, param_name: str, default: Optional[Any] = None):
        self._param_name = param_name
        self._default = default

    def get(self):
        """获取参数值，不存在返回None"""
        return rospy.get_param(self._param_name, self._default)

    def set(self, value: Any):
        """设置参数值"""
        rospy.set_param(self._param_name, value)


class ROSProxy:
    """ROS话题/参数代理类，简化属性访问"""

    def __getattribute__(self, name: str):
        # 调用父类方法安全获取属性，避免递归
        attr = super().__getattribute__(name)
        # 如果是话题字段，返回其处理后的值
        if isinstance(attr, rostopic_field):
            return attr.value
        # 如果是参数字段，返回其值
        if isinstance(attr, rosparam_field):
            return attr.get()
        # 普通属性直接返回
        return attr

    def __setattr__(self, name: str, value: Any):
        try:
            # 安全获取已存在的属性（避免触发__getattribute__的递归）
            attr = object.__getattribute__(self, name)
            # 如果是参数字段，调用set方法
            if isinstance(attr, rosparam_field):
                attr.set(value)
                return
        except AttributeError:
            # 属性不存在，直接设置
            pass
        # 普通属性/不存在的属性，调用父类方法设置，避免递归
        object.__setattr__(self, name, value)
