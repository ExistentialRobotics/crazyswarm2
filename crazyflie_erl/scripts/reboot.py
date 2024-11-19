import sys
from cflib.utils.power_switch import PowerSwitch

if len(sys.argv) != 2:
    print("Error: robot_id is missing")
    print("expected uri format is: radio://0/80/2M/E7E7E7E7E{robot_id}")
    print('Usage: {} uri'.format(sys.argv[0]))
    sys.exit(-1)

uri = "radio://0/80/2M/E7E7E7E7E" + str(sys.argv[1])
PowerSwitch(uri).stm_power_cycle()