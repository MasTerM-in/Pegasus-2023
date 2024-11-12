import time
import sys
from pymavlink import mavutil
from geopy import distance


# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Request all parameters
while True:
    time.sleep(0.01)
    try:    
      
        message = master.recv_match(type='GLOBAL_POSITION_INT',blocking=True).to_dict()

        wp1 = (message['lat']*10**-7,message['lon']*10**-7)

                   # INPUT Last WP
        wp2 = (-35.3672568,149.16094099999998)

        print('cheack')
        print(distance.distance(wp1, wp2).meters)

        if distance.distance(wp1, wp2).meters <= 2:
            print('reach WP4')
            print(distance.distance(wp1, wp2).meters)
            master.mav.command_long_send(master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,217, 6, 0, 0, 0, 0, 0)
            sys.exit(0)


    except Exception as error:
        print(error)
        sys.exit(0)


        


