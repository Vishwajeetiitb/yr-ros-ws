# JointCmd.msg

# Command Type Enum
int8 UNSET=0
int8 POSITION=1 # rad
int8 VELOCITY=2 # rad/s
int8 CURRENT=3  # TBD

# Fields
float32 cmd
int8 cmd_type