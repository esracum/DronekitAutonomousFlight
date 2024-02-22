from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import cv2
import dronekit


def detect_fire(video_path):
    ates_tespit = cv2.CascadeClassifier(
        '/home/raspberry/Desktop/ates/ates/fire_detection.xml')
    cap = cv2.VideoCapture(video_path)

    output = False
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()

    while (cap.isOpened):
        t1 = cv2.getTickCount()
        ret, frame = cap.read()
        if ret:
            blur = cv2.GaussianBlur(frame, (5, 5), 0)
            gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
            ates = ates_tespit.detectMultiScale(frame, 1.2, 3)
            if len(ates) > 0:
                output = True
                return output

            for (x, y, w, h) in ates:
                cv2.rectangle(frame, (x-20, y-20),
                              (x+w+20, y+h+20), (255, 0, 0), 2)
                roi_gray = gray[y:y+h, x:x+w]
                roi_color = frame[y:y+h, x:x+w]
                print("ates tespit edildi")

            cv2.putText(frame, 'FPS: {0:.2f}'.format(
                frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)
            t2 = cv2.getTickCount()
            time1 = (t2-t1)/freq
            frame_rate_calc = 1/time1

            yield frame

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap.release()


vehicle = connect('udp:127.0.0.1:14550')
print("baglanti kuruldu")

vehicle.mode = VehicleMode('AUTO')

vehicle.armed = True
print("arm edildi")


cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
print("okudu")


irtifa = 30
fire_detected = False
new_lat = 0
new_lon = 0
servo_number = 8
pwm_value_int = 2000

nextwaypoint = cmds.next

while True:
    # print("q ya bas")

    # a = input("deger gir\n")
    fire_detected = detect_fire(0)
    if fire_detected:
        print("ates tespit edildi")
        # fire_detected = True
        new_lat = vehicle.location.global_relative_frame.lat
        new_lon = vehicle.location.global_relative_frame.lon
        print("Vehicle's Latitude              =  ",
              vehicle.location.global_relative_frame.lat)
        print("Vehicle's Longitude             =  ",
              vehicle.location.global_relative_frame.lon)
        new_waypoint = LocationGlobalRelative(new_lat, new_lon, irtifa)
        cmd1 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                       0, 0, 0, 0, 0, 0, new_waypoint.lat, new_waypoint.lon, new_waypoint.alt)
        cmd2 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                       0, 0, 10, 0, 0, 0, new_waypoint.lat, new_waypoint.lon, new_waypoint.alt)
        cmd3 = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                       mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 0, servo_number, pwm_value_int, 0, 0, 0, 0, 0)

        #   cmds.clear()

        cmds.add(cmd1)
        cmds.add(cmd2)
        cmds.add(cmd3)
        cmds.upload()

        cmds.next = 7

        # if nextwaypoint == 9:
        #     vehicle.simple_goto(LocationGlobalRelative(
        #         vehicle.location.global_relative_frame.lat,
        #         vehicle.location.global_relative_frame.lon,
        #         vehicle.location.global_relative_frame.alt - 5
        #     ))

        #     vehicle.simple_goto(LocationGlobalRelative(
        #         vehicle.location.global_relative_frame.lat,
        #         vehicle.location.global_relative_frame.lon,
        #         vehicle.location.global_relative_frame.alt + 5
        #     ))

    break
