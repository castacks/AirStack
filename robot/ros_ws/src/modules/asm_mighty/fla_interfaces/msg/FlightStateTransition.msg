uint8 IDLE=0
uint8 INITIALIZING=1
uint8 TAKING_OFF=2
uint8 FOLLOWING_PATH=3
uint8 HOVERING=4
uint8 LANDING=5
uint8 RETURN_TO_LAUNCH=6
uint8 LANDED=7
uint8 TARGET_SEARCH=8
uint8 TARGET_APPROACH=9
uint8 LANDING_SEARCH=10
uint8 LANDING_APPROACH=11

std_msgs/Header header
std_msgs/String from_state
std_msgs/String to_state
uint8 from_state_id
uint8 to_state_id
