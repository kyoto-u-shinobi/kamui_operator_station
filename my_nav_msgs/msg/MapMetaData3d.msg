# This hold basic information about the characterists of the OccupancyGrid in 3D

# The map resolution [m/cell]
float32 resolution
# Map width [cells]
uint32 width
# Map depth [cells]
uint32 depth
# Map height [cells]
uint32 height
# The origin of the map [m, m, m].  This is the real-world position of the
# cell (0,0,0) in the map.
geometry_msgs/Point origin
