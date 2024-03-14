# JointCmd.msg

# Command Type Enum
int8 UNSET=0
int8 POSITION=1 # rad
int8 VELOCITY=2 # rad/s
int8 CURRENT=3  # TBD

# PowerState Enum
int8 STATE_UNSET=0 # No power state set - no state changed needed
int8 ENABLE=1      # Power is enabled
int8 DISABLE=2     # Power is disabled

# Fields
float32 cmd
int8 cmd_type
int8 enable_motor # Uses the PowerState Enum, Controls the Motor EStop, i.e. ENABLE - Motor Estop not toggled = motor enabled. Driverboard always on.
int8 enable_power # USes the PowerState Enum, Controls the power supply to the entire Motor and its driveboard.