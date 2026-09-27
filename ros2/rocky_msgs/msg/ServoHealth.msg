# Bus telemetry snapshot (SafetyMonitor publishes at 1-5 Hz)
std_msgs/Header header
uint8[] ids
float32[] temp_c
float32[] voltage_v
float32[] load_pct
float32[] current_a      # NaN for SCS0009 (no current sense)
uint8[] fault_bits       # status error bits per servo (rocky_driver.protocol.ERROR_BITS; 0 on SCS0009)
bool[] torque_cut        # true if the monitor limped this servo
