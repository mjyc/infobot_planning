# infobot_topo_mapping

## Demo
The demo files are dependent on `infobot_uw_world` package. However, the `infobot_uw_world` package is not listed in `package.xml` file with `run_depend` tag, because they are the optional components of this package.

To run a topo-mapping demo, run:
```
roslaunch infobot_topo_mapping test_extract_topology.launch
```
then, from a new shell, run:
```
rosrun infobot_topo_mapping create_new_map 0 0 0
```

## Internal Visualization Level
* `0` - No visualization.
* `1` - FinalProbabilityMap, PlaceMapSample, and MaxPlaveMapSample.
* `2` - Visualizations in `1` + ObstacleCost, ViewCost, and CenterCost.
* `3` - Visualizations in `2` + OccupancyGrid, Skeleton, InitialPlaceMapSample, ObstacleDistance, and CenterDistance.

## params.yaml File Description
* `min_obstacle_distance`
  * Minimum distance to obstacles. Should correspond to the robot's circumscribed radius as described here http://wiki.ros.org/costmap_2d/hydro/inflation. However, tune it to fir your domain.
* `optimal_view_dist`
* `min_voronoi_dist`
* `obstacle_cost_scaling`
* `view_cost_scaling`
* `center_cost_scaling`
* `obstacle_cost_weight`
  * A scaling factor to apply to cost values during obstacle inflation. Should correspond to the value defined here http://wiki.ros.org/costmap_2d/hydro/inflation which you use for navigation (a high value means the cost curve is steep, and vice versa).
* `view_cost_weight`
* `center_cost_weight`
