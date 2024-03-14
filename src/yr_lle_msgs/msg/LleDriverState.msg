# LLEDriverNode state
# Used to signal system warnings or errors detected by the driver.
# Usually stems from MCU.
# 

# Enum for error_codes
int32 ERR_UNSPECIFIED=0
int32 ERR_PLACEHOLDER_1=1
int32 ERR_PLACEHOLDER_2=2
int32 ERR_PLACEHOLDER_3=3

# Enum for error_code_event
int8 EVENT_UNSPECIFIED=0
int8 EVENT_SET=1
int8 EVENT_CLEAR=2

# Enum for error_code_side
int8 SIDE_UNSPECIFIED=-1
int8 SIDE_L=0
int8 SIDE_R=1

# Enum for error_code_loc
int8 LOC_UNSPECIFIED=-1
int8 LOC_PEL=0
int8 LOC_HIP=1
int8 LOC_KNE=2
int8 LOC_ANK=3
int8 LOC_FOT=4

# Flags
bool is_error               # True to signal at least one error is currently active

# The following is an array of errors along with more info about the error
int32[] error_codes               # A queue of error error_codes, arranged in time order, oldes to newest
int32[] error_codes_timestamp     # the triggered timestamp of each error code (seconds from 1970.1.1)
int8[] error_code_event           # the event type of each error code, one of the enums defined above
int8[] error_code_side            # associated body side, uses an enum defined above.
int8[] error_code_loc             # associated location, uses an enum defined above.
string[] error_msgs               # arbitrary error messages from driver to give more info about the event.

# Active errors
int32[] error_codes_active        # A pool of currently active error codes, the order does not carry meaning


