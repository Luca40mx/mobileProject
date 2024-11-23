#!/bin/bash
# source /opt/ros/foxy/setup.bash
/workspace/UnityHub.AppImage --no-sandbox -- --headless
exec "$@"