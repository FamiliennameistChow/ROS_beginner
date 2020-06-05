# This represents a 2-D elevation map, in which each cell represents the height of
# occupancy. (copy to OccupancyGrid.msg)

Header header 

#MetaData for the map
nav_msgs/MapMetaData info

# The map data, in row-major order, starting with (0,0). 
# height of the cell
int16[] data 
