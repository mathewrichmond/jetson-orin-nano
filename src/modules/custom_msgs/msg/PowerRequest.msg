# PowerRequest.msg
# Request to control Jetson GPU power state

std_msgs/Header header

# Power state request
string requested_state          # "on", "off", "sleep", "wake"

# Requestor information
string requester_node           # Name of node requesting power change
string reason                   # Reason for power request (e.g., "vision_needed", "battery_low")

# Priority level
int32 priority                  # Higher priority requests take precedence (0-10)

# Timeout
float32 timeout_sec             # How long to stay in requested state (0 = indefinite)
