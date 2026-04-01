# ImagingSonar message
# Contains timestamp, bins information, and image data

std_msgs/Header header

# Number of azimuth bins
int32 bins_azimuth

# Number of range bins
int32 bins_range

# Raw sonar image (as received)
sensor_msgs/Image raw_image

# Ground-truth intensity (float, same size as raw)
sensor_msgs/Image intensity

# Ground-truth elevation (per-pixel angle or height)
sensor_msgs/Image elevation

# Ground-truth 3D points
sensor_msgs/PointCloud2 point_cloud