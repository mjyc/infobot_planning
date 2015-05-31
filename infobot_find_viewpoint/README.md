 infobot_find_viewpoint

# Running Test/Demo Code

## test_compute_vis_val and test_compute_vis_vals

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

## test_find_viewpoint.launch

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
