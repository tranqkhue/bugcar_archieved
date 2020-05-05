Bugcar ROS Navigation Stack
====================
## What is this repo?
A 2D navigation stack that takes in information from odometry, sensor
streams, OccupancyGrid from *Perception Stack*, and a goal pose and outputs safe velocity commands that are sent
to a mobile base.
## How to run?
The main launch files should be in move_base package. These launch files can launch other nodes and packages
Example: `roslaunch move_base barebone_move_base.launch`
