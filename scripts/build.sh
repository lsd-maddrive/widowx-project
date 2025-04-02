#!/bin/bash

catkin build \
    arbotix_ros \
    widowx_arm \
    widowx_arm_bringup \
    widowx_arm_controller \
    widowx_arm_description \
    widowx_arm_moveit_config \
    -j$(($(nproc)-2))

# widowx_arm_ikfast_plugin
# widowx_block_manipulation
