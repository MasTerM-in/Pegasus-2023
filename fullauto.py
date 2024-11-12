from pymavlink import mavutil
from pymavlink import mavwp
import time
import sys
from geopy import distance
import math
                                        # Start a connection listening on a UDP port
master = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

# Wait for the first heartbeat 
master.wait_heartbeat()

print("Heartbeat from system (system %u component %u)" % 
        (master.target_system, master.target_component))

# write waypoint
wp = mavwp.MAVWPLoader()
def cmd_set_home(home_location, altitude):
    print('--- ', master.target_system, ',', master.target_component)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude) 

def uploadmission(aFileName):
    home_location = None
    home_altitude = None

    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:   
                linearray=line.split('\t')
                ln_seq = int(float(linearray[0]))
                ln_current = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_x=float(linearray[8])
                ln_y=float(linearray[9])
                ln_z=float(linearray[10])
                ln_autocontinue = int(float(linearray[11].strip()))
                if(i == 1):
                    home_location = (ln_x,ln_y)
                    home_altitude = ln_z
                p = mavutil.mavlink.MAVLink_mission_item_message(master.target_system, master.target_component, ln_seq, ln_frame,
                                                                ln_command,
                                                                ln_current, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_x, ln_y, ln_z)
                wp.add(p)
                    
    # cmd_set_home(home_location,home_altitude)
    # msg = master.recv_match(type = ['COMMAND_ACK'],blocking = True)
    # print(msg)
    # print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    # time.sleep(1)

    #send waypoint to airframe
    master.waypoint_clear_all_send()
    master.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        master.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))

#mode guide
def guided():
    master.mav.command_long_send(master.target_system, master.target_component,mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
    0,217, 4, 0, 0, 0, 0, 0)

#mode arm throttle
def arm():
    master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)


    msg = master.recv_match(type='COMMAND_ACK',blocking=True)
    print(msg)


#takeoff
def takeoff(alt):
    master.mav.command_long_send(master.target_system, master.target_component,                    # select ATTITUDE >= first WP
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

    msg = master.recv_match(type='COMMAND_ACK',blocking=True)
    print(msg)


guided()

arm()

takeoff(100)

#write waypoint ---> re_wp/file.txt
uploadmission('re_wp/re_simWP.txt')
time.sleep(1)

while True:
    time.sleep(0.05)
    try:
                                            # select 'mavpackattype'
        message = master.recv_match(type='GLOBAL_POSITION_INT',blocking=True).to_dict()
                        # select 'parameter'
        x=message['relative_alt']/1000
        print('Alt')
        print(x)

        # Check ATTITUDE
        if x>=95:
            
            #mode AUTO
            master.mav.command_long_send(master.target_system, master.target_component,mavutil.mavlink.MAV_CMD_DO_SET_MODE,
             0,217, 3, 0, 0, 0, 0, 0)
            break
            
    except Exception as error:
        print(error)
        break


while True:
    time.sleep(0.05)
    try:    
      
        message = master.recv_match(type='MISSION_CURRENT',blocking=True).to_dict()
        seq = message['seq']
        
        if seq != 0:
            message = master.recv_match(type='GLOBAL_POSITION_INT',blocking=True).to_dict()

            # Current WP
            wpC = (message['lat'])*10**-7,(message['lon'])*10**-7
            wpC1 = (message['lat'])*10**-7,(message['lon'])*10**-7,(message['relative_alt']/1000)
            latlon = str(wpC1)
            f = open('latlon.txt','w')
            f.write(latlon)
            f.close
            print(wpC)
                    
                    # INPUT---> WP/flie.txt 
            f = open("wp/simWP.txt", "r")
            x=f.readlines()
            dic ={}
            n=1
            for i in x:
                _data = i.split()
                dic[n] = {'lat':float(_data[0]),'lon':float(_data[1]),'alt': float(_data[2]), }
                n+=1

            list =[]
            list.append(dic[seq]['lat'])
            list.append(dic[seq]['lon'])
            wp=tuple(list)

            print('Reach WP:' + str(seq))

            #calculate 3d distance         #   (x2-x1)**2     +    (y2-y1)**2
            dis_3d = math.sqrt(distance.distance(wpC,wp).m**2 + (dic[seq]['alt']-message['relative_alt']/1000)**2)

            print(dis_3d)

        
            # Check last waypoint  
            if seq == len(dic):

            
                            # WPNAV_RADIUS               
                if dis_3d <= 2:
                    z ='Finish'
                    print(z)
                            # write  text for next mission
                    f = open('finish.txt','w')
                    f.write(z)
                    f.close
                    break

                  
    except Exception as error:
        print(error)
        sys.exit(0)




