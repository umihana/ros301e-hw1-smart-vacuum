# Telemetry 
std_msgs/Header header  # time stamp+frame_id

# 1) battery level in %
float32 battery_level  # 0.0-100.0

# 2) dustbin fullness status
# "enum" for dustbin_status, constants
uint8 DUSTBIN_OK=0
uint8 DUSTBIN_FULL=1
uint8 dustbin_status

# 3) cliff/bumper & proximity sensor state
bool cliff  # true = "floor edge"
bool bumper  # true = obsticle touched 
float32 proximity # distance to the nearest obsticle [m]
