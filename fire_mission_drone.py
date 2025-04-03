from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import cv2

# Bağlantıyı başlat
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
print("Bağlantı kuruldu")

# Otonom moda geçiş ve arm işlemi
vehicle.mode = VehicleMode('AUTO')
vehicle.armed = True

print("Arm edildi")

# 80 metreye yükselme
irtifa = 80
vehicle.simple_takeoff(irtifa)

print(f"{irtifa} metreye yükseliyor...")

# Komutları temizle ve güncelle
cmds = vehicle.commands
cmds.clear()

cmds.upload()
print("Komutlar yüklendi")

fire_detected = False

while True:
    a = input("Ateş tespit edildi mi? (q ile onayla): ")
    
    if a == "q":
        print("Ateş tespit edildi!")
        fire_detected = True
        new_lat = vehicle.location.global_relative_frame.lat
        new_lon = vehicle.location.global_relative_frame.lon

        print("Aracın Konumu:")
        print(f"Latitude: {new_lat}")
        print(f"Longitude: {new_lon}")

        # Servo hareketi için MAVLink komutu
        servo_number = 9
        pwm_value_int = 2000
        msg = vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_number,
            pwm_value_int,
            0, 0, 0, 0, 0
        )
        vehicle.send_mavlink(msg)

        # Yeni görev noktası ekleme
        cmd1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                       mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 
                       0, 0, 0, 0, 
                       new_lat, new_lon, irtifa)
        
        cmds.clear()
        cmds.add(cmd1)
        cmds.upload()
        
        break  # Ateş tespit edildiğinde döngüyü sonlandır
