# Car state - each car publishes its own state here
LEADER_STATE = "cars/leader/state"
FOLLOWER_STATE = "cars/follower/state"

# Car commands - each car listens for commands here
LEADER_CMD = "cars/leader/cmd"
FOLLOWER_CMD = "cars/follower/cmd"

# System wide
SYSTEM_MODE = "system/mode"           # "manual" or "autonomous"
MANUAL_TARGET = "system/manual/target" # "leader" or "follower"