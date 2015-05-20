# Includes the timestamp of last update and sequential
# number that is increased every update
std_msgs/Header header

# ID of the topological map
string topo_map_id

# Array of places in the topological map
TopologicalPlace[] places
