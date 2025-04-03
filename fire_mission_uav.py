from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import cv2

# Bağlantı kur

vehicle = connect('udp:127.0.0.1:14550')
print("baglantı kuruldu")

# Cihazı arm etmek için hazırlayın
# while not vehicle.is_armable:
#     print("Cihaz hazırlanıyor. Lütfen bekleyin...")
#     time.sleep(1) (simülasyonda var)


# Cihazı arm et
vehicle.mode = VehicleMode('STABILIZE') #? auto mu olmalı
vehicle.armed = True
print("arm edildi")

vehicle.simple_takeoff(15)
print("15 metreye yukseldi")

vehicle.mode = VehicleMode('AUTO')
print("AUTO mod başladı")

# Hedef konumları listeye ekleyin
# Waypoint'leri tanımla
waypoints = [
    LocationGlobalRelative(39.1234, 32.5678, 10),  # waypoint 1
    LocationGlobalRelative(39.1235, 32.5679, 10),  # waypoint 2
    LocationGlobalRelative(39.1236, 32.5680, 10),  # waypoint 3
    LocationGlobalRelative(39.1237, 32.5681, 10),  # waypoint 4
    LocationGlobalRelative(39.1238, 32.5682, 10),  # waypoint 5
    LocationGlobalRelative(39.1239, 32.5683, 10),  # waypoint 6
    LocationGlobalRelative(39.1240, 32.5684, 10)   # waypoint 7
]

# Komutları oluştur
cmds = vehicle.commands
cmds.clear()
print("okudu")

for i in range(len(waypoints)):
    print("bas")
    wp = waypoints[i]
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)
    print("son")

# Komutları araca yükle
#cmds.upload()
print("okudu")
# ates_tespit = cv2.CascadeClassifier('C:\\Users\\windows\\Desktop\\ates\\fire_detection.xml')
# #burda xml dosyasının bulundugu local konum girilmeli
# cap = cv2.VideoCapture(0)

# fire_detected = False

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
          #break

#  while dongusu bitecek 

print("commandler yuklendi")
fire_detected =False
while True:
    print("q y bas")
    print("3 saniye içinde q ya bas")
    
    a = input("değer gir")
    # tus = cv2.waitKey(10000)
    if  a == "q":
        print("ates tespit edildi")
        fire_detected = True
    break
print(fire_detected)
if fire_detected: #tabları ayarla
   print("Yangın tespit edildi! Drone moduna geçiliyor...")
   msg = vehicle.message_factory.command_long_encode(
   0, 0, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, 0, 1, 0, 0, 0, 0, 0, 0)
   vehicle.send_mavlink(msg)
   time.sleep(2)
   print("5 metre alçılıyor...")
   vehicle.simple_goto(LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        vehicle.location.global_relative_frame.alt - 5
    ))
time.sleep(3)
            
servo_num = 9  
angle = 90  

msgg = vehicle.message_factory.command_long_encode(
    0, 0,    # hedef sistem ve bileşen ID'si
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, # MAV_CMD_DO_SET_SERVO komutu
    0, # tekrarlanma sayısı
    servo_num, # Servo motorunun numarası
    angle, # Servo motorunun açısı
    0, 0, 0, 0, 0) # diğer parametreler
vehicle.send_mavlink(msgg)

             
    

print("5 metre yükseliyor...")
vehicle.simple_goto(LocationGlobalRelative(
    vehicle.location.global_relative_frame.lat,
    vehicle.location.global_relative_frame.lon,
   vehicle.location.global_relative_frame.alt + 5
   ))
time.sleep(2)
print("Sabit kanat moduna geçiliyor...") 





vehicle.mode = VehicleMode('RTL') 
print("eski yerine döndü")
vehicle.close()
