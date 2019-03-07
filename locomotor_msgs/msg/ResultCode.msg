# This message contains three separate pieces.
#    A) A code indicating which component(s) the error originates from (bitmask style)
#    B) A code corresponding with the result_code defined in nav_core2/s.h
#    C) A freeform string message

# The enumerations below are not necessarily the exclusive values for the codes.
# Others may implement additional values beyond the ones shown, using custom state machines.

########### Component Values ###############################################
int32 GLOBAL_COSTMAP = 1
int32  LOCAL_COSTMAP = 2
int32 GLOBAL_PLANNER = 4
int32  LOCAL_PLANNER = 8

########### Result Codes ###################################################
int32 GENERIC_COSTMAP=0
int32 COSTMAP_SAFETY=1
int32 COSTMAP_DATA_LAG=2
int32 GENERIC_PLANNER=3
int32 GENERIC_GLOBAL_PLANNER=4
int32 INVALID_START=5
int32 START_BOUNDS=6
int32 OCCUPIED_START=7
int32 INVALID_GOAL=8
int32 GOAL_BOUNDS=9
int32 OCCUPIED_GOAL=10
int32 NO_GLOBAL_PATH=11
int32 GLOBAL_PLANNER_TIMEOUT=12
int32 GENERIC_LOCAL_PLANNER=13
int32 ILLEGAL_TRAJECTORY=14
int32 NO_LEGAL_TRAJECTORIES=15
int32 PLANNER_TF=16

########### Actual Data ####################################################
int32 component
int32 result_code
string message
