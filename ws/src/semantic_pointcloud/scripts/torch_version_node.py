#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _venv_bootstrap import activate

activate(__file__)

import rospy
import torch


def main():
    rospy.init_node("torch_version_node")
    rospy.loginfo("torch version: %s", torch.__version__)


if __name__ == "__main__":
    main()
