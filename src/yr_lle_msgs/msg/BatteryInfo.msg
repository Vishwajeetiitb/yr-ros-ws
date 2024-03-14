float32 voltage    # Battery voltage in volts
float32 soc        # State of Charge in percentage (0-100)

# Enum for battery status
uint8 UNKNOWN=0
uint8 CHARGING=1
uint8 DISCHARGING=2
uint8 FULL=3
uint8 IDLE=4

uint8 status       # Battery status as an enum