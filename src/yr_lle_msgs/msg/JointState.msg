# JointState.msg
std_msgs/Header header
float32 pos    # Motor Position (rad)
float32 vel    # Motor Velocity (rad/s)
float32 cur    # Motor Current (Amps)
float32 pos_encoder     # joint angle endoder position (radians)
int16 encoder  # Joint angle Encoder value
bool is_enabled # is joint enabled (estop) - false means a estop is activated.
bool is_powered # is joint powered (a hardware switch) - false means entire driveboard and motor is powered off
