# GarbageArray.msg
# Array of detected garbage items

std_msgs/Header header

# Array of garbage items
GarbageItem[] items

# Zone this detection came from
string zone_id

# Total count by type
uint32 count_small
uint32 count_large
uint32 count_floating

