from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import cv2
import dronekit


vehicle = connect('udp:127.0.0.1:14550')

print("baglanti kuruldu")

vehicle.mode = VehicleMode('AUTO') 
vehicle.armed = True

print("arm edildi")

irtifa = 80
vehicle.simple_takeoff(irtifa)

print("20 metreye yukseldi")

cmds = vehicle.commands
cmds.clear()

print("okudu")


cmds.upload()
print("okudu")

fire_detected =  False
new_lat=0
new_lon=0
while True :
    print("3 saniye icinde q ya bas")
    
    a = input("deger gir")
    tus = cv2.waitKey(1)
    if  a == "q":
        print("ates tespit edildi")
        fire_detected = True
        new_lat = vehicle.location.global_relative_frame.lat * 1e7
        new_lon = vehicle.location.global_relative_frame.lon * 1e7
        print("Vehicle's Latitude              =  ", vehicle.location.global_relative_frame.lat)
        print("Vehicle's Longitude             =  ", vehicle.location.global_relative_frame.lon)
        servo_number = 9
        pwm_value_int = 2000
        cmd1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
	      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
	      new_lat, new_lon, irtifa)
        msg = vehicle.message_factory.command_long_encode(
		0, 0, 
		mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
		0,
		servo_number,
		pwm_value_int,
		0,0,0,0,0
		)
    vehicle.send_mavlink(msg)
    vehicle.commands.clear()
    cmds.add(cmd1)
    vehicle.commands.add(cmd1)
    vehicle.commands.upload()
    break
    

    






