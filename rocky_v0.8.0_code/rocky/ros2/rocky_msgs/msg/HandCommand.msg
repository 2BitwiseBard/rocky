# Transforming hand control (foot <-> hand is a MODE, not just an angle)
uint8 MODE_FOOT=0        # fingers closed into the walking cone
uint8 MODE_HAND=1        # gripper active

uint8 hand               # 0..4
uint8 mode
float32 open_frac        # 0..1 (driver maps to 0..55 deg until hand v0.3)
