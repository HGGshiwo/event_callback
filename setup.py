from setuptools import setup, find_packages
from pathlib import Path


requirements = (
    Path(__file__)
    .parent.joinpath("event_callback", "requirements.txt")
    .read_text(encoding="utf-8")
    .split("\n")
)

setup_kwargs = dict(
    name="event_callback",  # 外部导入的Python包名，和event_callback/目录一致！
    version="0.0.0",
    author="Your Name",
    author_email="your_email@xxx.com",
    description="event callback package",
    long_description="",
    url="",  # 你的仓库地址（可选）
    # 自动查找event_callback/下的所有Python模块（含子包）
    packages=find_packages(exclude=["scripts", "launch", "test"]),
    # 声明Python包的依赖（和requirements.txt一致，可选）
    install_requires=requirements,
    package_dir={"": "src"},
    python_requires=">=3.6",
)

setup(**setup_kwargs)
