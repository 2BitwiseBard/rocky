# Omnidirectional gait command (BODY frame — Pebble has no front, D005)
uint8 MODE_IDLE=0
uint8 MODE_WALK=1        # 5-leg wave
uint8 MODE_ARMED_WALK=2  # 4-leg wave, arm_legs raised (gait v0.2)
uint8 MODE_MANIP=3       # 3-leg stance manipulation (D013/D018)
uint8 MODE_SLEEP=4       # belly down, torque off (thermal safety)

uint8 mode
float32 vx               # m/s, body +X
float32 vy               # m/s, body +Y
float32 wz               # rad/s, +CCW
uint8[] arm_legs         # which legs are arms in ARMED_WALK/MANIP
