#!/bin/bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Iamkewl

set -e

export LIBGL_ALWAYS_SOFTWARE=0
export MESA_GL_VERSION_OVERRIDE=3.3

xhost +local:docker
docker compose up -d --build
docker exec -it moveit_humble_dev bash -lc "if [ ! -e /root/Humble_WS ]; then ln -s /root/colcon_ws /root/Humble_WS; fi; if [ ! -e /root/humble_ws ]; then ln -s /root/colcon_ws /root/humble_ws; fi; source /opt/ros/humble/setup.bash; if [ -f /root/colcon_ws/install/setup.bash ]; then source /root/colcon_ws/install/setup.bash; fi; exec bash"
