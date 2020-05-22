# Bugcar Workspace
[![Build Status](https://travis-ci.com/tranqkhue/bugcar.svg?token=Lywx1Kzgk1pQDGSsYXzR&branch=master)](https://travis-ci.com/tranqkhue/bugcar)
# About
This repo is Bugcar Workspace, which includes the neccessary source codes and launch files for the Bugcar Autonomous Outdoor Rover
# Important
All the launch files are in **bring_up** folder

For Python-based package, please add [Shebang](https://en.wikipedia.org/wiki/Shebang_%28Unix%29#Portability) at the top of the Python src scripts

- `#!/usr/bin/env python2`
- `# -*- coding: utf-8 -*-`

And also make sure that Python scripts are executable of the error `Couldn't find executable named` would happen!

Make sure the nodes in the workspace following rules:
- Correct topics
- Correct message types
- Correct frame
# This Workspace currently has:
- Bugcars' Robot Localization (based on [ROS robot_localization](https://github.com/cra-ros-pkg/robot_localization))
- Custom *Navigation Stack* which partially compartible with ROS move_base navigation stack
- Bugcar's bringup launch files
