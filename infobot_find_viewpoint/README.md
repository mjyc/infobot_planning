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

First, collect data from the robot using below command:
```
roslaunch infobot_find_viewpoint test_retrospective_collect_data.launch
```

Save the path and octomap using the command below:
```
roslaunch infobot_find_viewpoint test_retrospective_save_data.launch
```
Note that the above command requires ~/bags/ and ~/octomap/ folders. Also kill the first command using Ctrl-C (otherwise it will continue collect data in the bag file).

Launch below command to see if a proper image was retrieved:
```
roslaunch infobot_find_viewpoint test_retrospective_collect_data.launch
```
