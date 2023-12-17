time time         # rospy.Time, ros::Time

float64 speed  # m/s
float64 yawrate   # rad/s

float64 ax # m/s^2
float64 ay # m/s^2

float64 steering_angle   # deg
float64 steering_torque  # Nm
float64 steering_rate    # deg/s

uint8 autonomous_mode
uint8 OFF = 0
uint8 LONGITUDINAL_ONLY = 1
uint8 LATERAL_ONLY = 2
uint8 AUTONOMOUS = 3

string gear_position
uint8 driver_override
uint8 brake_act
string cruise_switch_state

bool right_turn_signal
bool left_turn_signal
bool hazard_signal
