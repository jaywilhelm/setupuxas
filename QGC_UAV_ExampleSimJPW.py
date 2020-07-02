import struct, array, os, time, sys
import argparse

# For MAVLink
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink
import numpy as np
# For Dubins Vehicle
from dubinsUAV import dubinsUAV

if(len(sys.argv) == 2):
    sendport = int(sys.argv[1])
else:
    sendport = 14550
connstring = 'udpout:localhost:'+str(sendport)
print('Sending to ' + connstring)
UAVList = []

dt=1
simUAV = {'position': (45.32, -120.97), 'velocity': 0.0001, 'heading': np.deg2rad(270)}
simUAV = dubinsUAV(position=simUAV['position'], velocity=simUAV['velocity'], heading=simUAV['heading'], dt=dt)

uavx = {}
uavx['dubins'] = simUAV
uavx['altitude'] = 100
uavx['id'] = 4
uavx['mavlink'] = mavutil.mavlink_connection(connstring, source_system=uavx['id'], source_component=1)
UAVList.append(uavx)
# if you need another vehicle
# simUAV = {'position': (39.4, -82.15), 'velocity': 0.0001, 'heading': 0}
# simUAV = dubinsUAV(position=simUAV['position'], velocity=simUAV['velocity'], heading=simUAV['heading'], dt=dt)
# uavx = {}
# uavx['dubins'] = simUAV
# uavx['altitude'] = 100
# uavx['id'] = 10
# uavx['mavlink'] = mavutil.mavlink_connection(connstring, source_system=uavx['id'], source_component=1)
# UAVList.append(uavx)

while True:
    for uavobj in UAVList:
        uav = uavobj['dubins']
        uav.update_pos()
        master = uavobj['mavlink']
        #type, autopilot, base_mode, custom_mode, system_status
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_FIXED_WING,
                    mavutil.mavlink.MAV_TYPE_GENERIC, 128, 0, 12)   

        #time_boot needs to be real
        time_boot = 0
        ALT = uavobj['altitude']
        heading = int(np.rad2deg(uav.heading)*100)#watch this, degree or radians..
        print('ID: ' + str(uavobj['id']) + '\tLat: ' + str(uav.x) + ' Lon: ' + str(uav.y) + ' Heading: ' + str(heading))
        master.mav.global_position_int_send(time_boot_ms=int(time_boot),
                                            lat=int(uav.x*1e7),
                                            lon=int(uav.y*1e7),
                                            alt=int(ALT*1e3), relative_alt=int(ALT*1e3),
                                            vx=int(uav.vx*1e2),  vy=int(uav.vy*1e2), vz=0,
                                            hdg=heading)

        #onboard_control_sensors_present, onboard_control_sensors_enabled, onboard_control_sensors_health, load, voltage_battery, current_battery, battery_remaining, drop_rate_comm, errors_comm, errors_count1, errors_count2, errors_count3, errors_count4
        master.mav.sys_status_send( 1, 1 ,1 ,500 , 12000 ,-1 , -1 ,0 , 0, 0, 0, 0, 0)

    time.sleep(dt)


