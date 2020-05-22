# Bugcar configuration for mapviz visualization tool
## Installation and basic configuration

Please refer at the [original swri-robotics/mapviz](https://github.com/swri-robotics/mapviz/)

## Bing Map API key

Bing Map is free for small testing and integrated into Mapviz, which means no harassed installation

### THIS API KEY IS FOR TESTING ONLY! PLEASE DO NOT ABUSE IT

Bing Map API key is `_fMWQEW8yR_h2aeA92mes5Th6i3Ch1RRmER6gSyftm36PZZIfKeny_`

## Origin points of Mapviz

Mapviz use */initialize_origin/local_xy_origins* rosparam to set its map origin.
For our purpose, that *origin* is taken from *datum* of *robot_localization* via a Python script