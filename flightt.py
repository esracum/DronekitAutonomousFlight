from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
from math import radians, sin, cos, sqrt,atan2
import time

# Bağlantı kur
vehicle = connect('udp:127.0.0.1:14550')

while vehicle.is_armable != True:
        print("Arm edilemiyor.")
        time.sleep(1)
    
print("Arm ediliyor.")
vehicle.armed = True

# Modu 'AUTO' olarak ayarla
vehicle.mode = VehicleMode("AUTO")

# Waypoint'leri tanımla
waypoints = [
    LocationGlobalRelative(-35.2227644,149.1572571, 10),
    LocationGlobalRelative( -35.2233078,149.1559267,10),
    LocationGlobalRelative(-35.2240966,149.1562700,10),
    LocationGlobalRelative(-35.2241141,149.1575360,10)

]

# Uçuş görevini yükle
cmds = vehicle.commands
cmds.clear()
for wp in waypoints:
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                      wp.lat, wp.lon, wp.alt))
cmds.upload()

print("commandler yüklendi")

# Uçuş görevini başlat
vehicle.armed = True

#aradaki uzaklık fonksiyonunu ilkle

def get_distance_metres(aLocation1, aLocation2):
    dlat = radians(aLocation2.lat - aLocation1.lat)
    dlong = radians(aLocation2.lon - aLocation1.lon)
    a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(aLocation1.lat)) \
        * cos(radians(aLocation2.lat)) * sin(dlong / 2) * sin(dlong / 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    d = 6371000 * c
    return d

# Waypoint'leri uygula ve tamamla
for wp in waypoints:
    vehicle.simple_goto(wp)
    print("kontrol")
while True :
    print("kontrol2")
    distance_remaining = get_distance_metres(vehicle.location.global_relative_frame, wp)
    if distance_remaining <= 10.0:  # 10 santimetre mesafe
       break
    time.sleep(1)

print("Waypointler uygulandıve tamamlandı")


# while(True):
#     ret, frame = cap.read()
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     ates = ates_tespit.detectMultiScale(frame, 1.2, 3)

#     for (x,y,w,h) in ates:
#         cv2.rectangle(frame,(x-20,y-20),(x+w+20,y+h+20),(255,0,0),2)
#         roi_gray = gray[y:y+h, x:x+w]
#         roi_color = frame[y:y+h, x:x+w]
#         print("yangin tespit edildi")
#         fire_detected = True
print("qya basın")

if cv2.waitKey(1) & 0xFF == ord('q'):
      fire_detected = True



if fire_detected:
      print("Yangın tespit edildi! Drone moduna geçiliyor...")
      vehicle.mode = VehicleMode("QLOITER")
      time.sleep(2)
      print("5 metre alçılıyor...")
      vehicle.simple_goto(LocationGlobalRelative(
          vehicle.location.global_relative_frame.lat,
          vehicle.location.global_relative_frame.lon,
          vehicle.location.global_relative_frame.alt - 5
      ))
      time.sleep(3)
      print("5 metre yükseliyor...")
      vehicle.simple_goto(LocationGlobalRelative(
          vehicle.location.global_relative_frame.lat,
          vehicle.location.global_relative_frame.lon,
          vehicle.location.global_relative_frame.alt + 5
      ))
      time.sleep(2)
      print("Sabit kanat moduna geçiliyor...")
      vehicle.mode = VehicleMode("FBWA")

# Bağlantıyı kapat
vehicle.close()


# Uçuşu sonlandır
vehicle.mode = VehicleMode("LAND")
vehicle.close()
