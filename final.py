from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time

# Bağlantı kur
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Cihazı arm etmek için hazırlayın
# while not vehicle.is_armable:
#     print("Cihaz hazırlanıyor. Lütfen bekleyin...")
#     time.sleep(1) (simülasyonda var)


# Cihazı arm et
vehicle.mode = VehicleMode('AUTO') #? auto mu olmalı
vehicle.armed = True

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

for i in range(len(waypoints)):
    wp = waypoints[i]
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

# Komutları araca yükle
cmds.upload()
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
while True:
    tus = cv2.waitKey(1)
    if  tus==ord("q"): 
        print("ates tespit edildi")

    
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
vehicle.mode = VehicleMode("FBWA") 



vehicle.mode = VehicleMode('RTL') 

vehicle.close()


'''
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import cv2

# Drone'a bağlan
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# **Kalkış Fonksiyonu**
def arm_and_takeoff(target_altitude):
    print("İniş takımları açık, arm ediliyor...")
    while not vehicle.is_armable:
        print("Drone hazırlık yapıyor, lütfen bekleyin...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")  # Kontrollü uçuş modu
    vehicle.armed = True
    while not vehicle.armed:
        print("Arm işlemi bekleniyor...")
        time.sleep(1)

    print("Kalkış yapılıyor...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(f"Güncel irtifa: {vehicle.location.global_relative_frame.alt:.2f} m")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Hedef irtifaya ulaşıldı!")
            break
        time.sleep(1)

# **Waypoints Tanımla**
waypoints = [
    LocationGlobalRelative(39.1234, 32.5678, 10),
    LocationGlobalRelative(39.1235, 32.5679, 10),
    LocationGlobalRelative(39.1236, 32.5680, 10),
    LocationGlobalRelative(39.1237, 32.5681, 10),
    LocationGlobalRelative(39.1238, 32.5682, 10),
    LocationGlobalRelative(39.1239, 32.5683, 10),
    LocationGlobalRelative(39.1240, 32.5684, 10)
]

# **Komutları Güncelle**
cmds = vehicle.commands
cmds.clear()

for wp in waypoints:
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                  mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                  wp.lat, wp.lon, wp.alt)
    cmds.add(cmd)

cmds.upload()

# **Kalkış yap**
arm_and_takeoff(10)

# **AUTO moda geçerek waypoints'e git**
vehicle.mode = VehicleMode('AUTO')

# **Yangın Algılama Fonksiyonu**
fire_cascade = cv2.CascadeClassifier('/home/raspberry/Desktop/ates/ates/fire_detection.xml')
cap = cv2.VideoCapture(0)  # Kameradan video al

fire_detected = False

while True:
    ret, frame = cap.read()
    if not ret:
        print("Kamera verisi alınamıyor!")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    fires = fire_cascade.detectMultiScale(gray, 1.2, 3)

    for (x, y, w, h) in fires:
        cv2.rectangle(frame, (x-20, y-20), (x+w+20, y+h+20), (255, 0, 0), 2)
        print("Yangın tespit edildi!")
        fire_detected = True
        break

    cv2.imshow("Kamera", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    if fire_detected:
        break

cap.release()
cv2.destroyAllWindows()

# **Yangın tespit edildiğinde müdahale**
if fire_detected:
    print("Yangın tespit edildi! VTOL moduna geçiliyor...")
    msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, 0, 1, 0, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    time.sleep(2)

    print("5 metre alçalıyor...")
    new_altitude = max(vehicle.location.global_relative_frame.alt - 5, 2)  # Minimum 2 metreye kadar in
    vehicle.simple_goto(LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        new_altitude
    ))
    time.sleep(3)

    # **Servo Motor Hareketi**
    servo_num = 9  
    pwm_value = 1500  # Açılış açısı için uygun PWM değeri
    print("Servo hareketi başlatılıyor...")

    msgg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, servo_num, pwm_value, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msgg)

    time.sleep(2)

    print("5 metre yükseliyor...")
    vehicle.simple_goto(LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        vehicle.location.global_relative_frame.alt + 5
    ))
    time.sleep(3)

    print("Sabit kanat moduna geçiliyor...")
    vehicle.mode = VehicleMode("FBWA")

# **Görevi Tamamla ve Eve Dön**
print("RTL (geri dönüş) başlatılıyor...")
vehicle.mode = VehicleMode('RTL')

vehicle.close()
'''
