# Bugcar Start Sequence
 - Launch rc_node only:
	- roslaunch bugcar_bringup rc_only.launch
 - Launch standard localization:
	- roslaunch bugcar_bringup test_run_rc.launch
 - Launch map_viz node:
	- roslaunch bugcar_mapviz mapviz.launch
 - Launch move_base node:
	- roslaunch bugcar_bringup osm_move_base.launch
 
 - **IMPORTANT:**
	- Check covariance of gps (rostopic echo gps/fix)
	- Check if move\_base/cmd\_vel and rc\_controller/cmd_vel are running
	- Check if twist\_mux has already subscribed to move\_base/cmd\_vel and rc\_controller/cmd_vel

- **MISC**
	- If the roadmap is not loaded properly in rviz, reset rviz and relaunch move_base node
	- Open rviz with the config file located at bugcar/bugcar_bringup/cfg/viz/RvizConfig.rviz
	- Open mapviz with the config file located at bugcar/bugcar_bringup/cfg/viz/bugcar.mvc
	- Make sure there is Internet Connection to use mapviz
	- If mapviz does not load properly, relaunch mapviz node
	- Param files are (mostly) located in bugcar/bugcar\_bringup/cfg/move_base

