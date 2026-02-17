#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 获取package.xml中的信息
setup_args = generate_distutils_setup(
    packages=['event_callback'],
    package_dir={'': '.'}, # 模块所在的根路径
)

setup(**setup_args)