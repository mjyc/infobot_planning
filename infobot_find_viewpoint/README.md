# infobot_find_viewpoint

## Running Test/Demo Code

### test_compute_vis_val and test_compute_vis_vals

Run
```
roslaunch infobot_find_viewpoint test_compute_vis.launch
```
then, from a new shell, run:
```
rosrun infobot_find_viewpoint test_compute_vis_val
```
or run:
```
rosrun infobot_find_viewpoint test_compute_vis_vals
```
You can experiment effects of parameters by using dynamic reconfigure to change parameters of find_viewpoint and visibility_reasoner nodes.

### test_find_viewpoint.launch

Run:
```
roslaunch infobot_find_viewpoint test_find_viewpoint.launch
```
Running above code will send couple goals and save result images in ~/images folder (you will need to create this folder).

If you want to send goals yourself, run
```
roslaunch infobot_find_viewpoint test_find_viewpoint.launch exclude_goals:=true
```
and run below code to send custom goals:
```
rosrun infobot_find_viewpoint send_goal sim-simple-room1.jpg 0 sim-simple-room1.yaml sim-simple-model.bt sim-simple_corridor-topo.yaml
```

### Retrospective Query Demo

Run:
```
roslaunch infobot_find_viewpoint test_retrospective_collect_data.launch
```
This command will start the morse simulation and move the robot to pre-selected waypoints.

Once the robot finishes navigation, run:
```
roslaunch infobot_find_viewpoint test_retrospective_save_data.launch
```
The above command requires ~/bags/ and ~/octomap/ folders, so make sure to create them before running the command. Also, make sure to kill the first command using Ctrl-C--otherwise it will continue collect data.

Finally, run:
```
roslaunch infobot_find_viewpoint test_retrospective_collect_data.launch
```
If everything went okay, it will produce a result image in test folder.
