# SEA foot-cartridge contact sensing (D010: microswitch = free contact sensor)
std_msgs/Header header
bool[5] in_contact
float32[5] spring_mm     # estimated compression (0..7 mm travel)
