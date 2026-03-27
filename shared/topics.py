# Car state - each car publishes its own state here
LEADER_STATE = "cars/leader/state"
FOLLOWER_STATE = "cars/follower/state"

# Car commands - each car listens for commands here
LEADER_CMD = "cars/leader/cmd"
FOLLOWER_CMD = "cars/follower/cmd"

# Servo commands
LEADER_SERVO = "cars/leader/servo"
FOLLOWER_SERVO = "cars/follower/servo"

# Buzzer
LEADER_BUZZER = "cars/leader/buzzer"
FOLLOWER_BUZZER = "cars/follower/buzzer"

# position
LEADER_POSITION  = "cars/leader/position"
FOLLOWER_POSITION = "cars/follower/position"

# waypoints
LEADER_WAYPOINT  = "cars/leader/waypoint"
FOLLOWER_WAYPOINT = "cars/follower/waypoint"

# System wide
SYSTEM_MODE = "system/mode"           # "manual" or "autonomous"
MANUAL_TARGET = "system/manual/target" # "leader" or "follower"

CUP_POSITIONS = "cups/positions"