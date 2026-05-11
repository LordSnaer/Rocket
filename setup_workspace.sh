#!/bin/bash

cd /home/rocket/rocket_ws

# Clone the template
git clone https://github.com/joshnewans/articubot_one.git src/articubot_one

# Initialize the ROS workspace
cd /home/rocket/rocket_ws

echo "ROS Workspace created at /home/rocket/rocket_ws"
echo "Template cloned to src/articubot_one"
echo ""
echo "To build the workspace, run:"
echo "  cd /home/rocket/rocket_ws"
echo "  colcon build"
