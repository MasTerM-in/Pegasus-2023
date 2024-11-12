import time
import sys

# Import mavutil
from pymavlink import mavutil


import time
import sys

# Import mavutil
from pymavlink import mavutil


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

while True:
    time.sleep(0.05)
    try:
                                            # select 'mavpackattype'
        message = master.recv_match(type='GLOBAL_POSITION_INT',blocking=True).to_dict()
                        # select 'parameter'
        x=message['relative_alt']/1000
        print(x)


        #select ATTITUDE
        if x>=100:
            master.mav.command_long_send(master.target_system, master.target_component,mavutil.mavlink.MAV_CMD_DO_SET_MODE,
             0,217, 3, 0, 0, 0, 0, 0)
            break




















    except Exception as error:
        print(error)
        sys.exit(0)








