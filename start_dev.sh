#!/bin/bash
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Iamkewl

set -e

xhost +local:docker
docker compose up -d --build
docker exec -it moveit_humble_dev bash -lc "source /opt/ros/humble/setup.bash; if [ -f /root/colcon_ws/install/setup.bash ]; then source /root/colcon_ws/install/setup.bash; fi; exec bash"
