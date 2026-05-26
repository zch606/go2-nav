std_msgs/Header header

uint8 NORMAL=0
uint8 DEGRADED=1
uint8 SLOW=2
uint8 PAUSE=3
uint8 STOP=4
uint8 MANUAL=5

uint8 state
uint8 severity
bool emergency_stop
bool allow_motion
float32 speed_scale
float32 terrain_cost
float32 roll_deg
float32 pitch_deg
float32 odom_covariance
string reason
