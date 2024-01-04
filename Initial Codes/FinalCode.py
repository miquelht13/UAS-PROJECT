from __future__ import print_function
from dronekit import connect, VehicleMode
import time
import tkinter as tk
from pymavlink import mavutil
import math
import threading
import numpy as np
import cv2

def arm_and_takeoff():
    status_label.config(text="Basic Pre-Arm checks")

    while not vehicle.is_armable:
        status_label.config(text="Basic Pre-Arm checks")
        time.sleep(1)

    status_label.config(text="Arming motors")
    #Drone should be armed in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    #Confirm vehicle armed
    while not vehicle.armed:
        status_label.config(text="Waiting for arming")
        time.sleep(1)

    status_label.config(text="Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    if True:
        update_info()


def exit_simulation():
    vehicle.close()
    root.destroy()


def RTL():
    vehicle.mode = VehicleMode("RTL")
    status_label.config(text="Coming back")


def goNorth():
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        speed,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    time.sleep(1)


def goSouth():
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        -speed,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    #time.sleep(10)


def goEast():
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        0,
        speed,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    #time.sleep(10)


def goWest():
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,
        0,
        0,
        0,
        -speed,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)
    #time.sleep(10)


def prueba():
    topsecret = True
    checkpoints = [0,0,0,0,0]
    n = 0
    while topsecret:  
        goNorth()
        time.sleep(5)
        checkpoints[n] = vehicle.location.global_frame
        print(checkpoints)
        n = n + 1

        goEast()
        time.sleep(5)
        checkpoints[n] = vehicle.location.global_frame
        print(checkpoints)
        n = n + 1

        goSouth()
        time.sleep(5)
        checkpoints[n] = vehicle.location.global_frame
        print(checkpoints)

        goWest()
        time.sleep(5)

        #entrega
        time.sleep(1)
        currentlocation = vehicle.location.global_relative_frame
        currentlocation.alt = 3
        vehicle.simple_goto(currentlocation)
        time.sleep(6)
        currentlocation = vehicle.location.global_relative_frame
        currentlocation.alt = 6
        vehicle.simple_goto(currentlocation)
        time.sleep(6)

        time.sleep(1)
        i = 0
        while i <= n:
            vehicle.simple_goto(checkpoints[n-i])
            currentlocaton = vehicle.location.global_frame
            dist = DistanceInMeters(checkpoints[n-i],currentlocaton)
            #Wait until reach point
            while dist > 0.5:
                time.sleep(0.25)
                currentlocaton = vehicle.location.global_frame
                dist = DistanceInMeters(checkpoints[n-i],currentlocaton)
            time.sleep(1)
            i = i +1

        RTL()

        topsecret = False


def DistanceInMeters(loc1,loc2):
    dlat = loc1.lat - loc2.lat
    dlon = loc1.lon - loc2.lon
    return math.sqrt((dlat*dlat)+(dlon*dlon))*1.113195e5


def update_info():
    if not vehicle.armed:
        status_label.config(text="Not armed")
    if vehicle.armed:
        info_label.config(text=f"Mode: {vehicle.mode.name}\n"
                               f"Altitude: {vehicle.location.global_relative_frame.alt}\n"
                               f"Heading: {vehicle.heading}")
    if vehicle.mode.name == 'RTL':
        if not vehicle.armed:
            status_label.config(text="Landed")
    if simulation_running:
        root.after(1000, update_info)


def colorDetector(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of red color in HSV
    lower_red = np.array([50, 50, 110])  # min HSV values of red
    upper_red = np.array([255, 255, 130])  # max HSV values of red
    # Threshold the HSV image to get only red colors
    mask_red = cv2.inRange(hsv, lower_red, upper_red)

    # define range of red color in HSV
    lower_blue = np.array([110, 50, 50])  # min HSV values of red
    upper_blue = np.array([130, 255, 255])  # max HSV values of red
    # Threshold the HSV image to get only red colors
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)


# Función para mover el servo
def move_servo(channel, pwm_value):
    # Enviar comando PWM al servo a través del canal específico
    vehicle.channels.overrides[channel] = pwm_value


servo_channel = 8  # AUX OUT 3 generalmente se asigna al canal 8
servo_pwm_value = 1500

connection_string = 'tcp:127.0.0.1:5762'
vehicle = connect(connection_string)
print("Connected")

# Variable de estado para controlar la simulación
simulation_running = True

speed = 1
aTargetAltitude = 6

root = tk.Tk()
root.title("Drone Information")

status_label = tk.Label(root, text="Waiting for data...")
info_label = tk.Label(root, text="Waiting for data...")
status_label.pack()
info_label.pack()

# Res camara: 640x480 -> centro: 320,240
# Cuadrado de X = (295,355), y=(215,265)
xI,yI,xF,yF=295,215,355,265

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
#frame = cv2.imread('image.jpg')
img=frame[yI:yF,xI:xF,:]

Prueba = threading.Thread(target=prueba)
# Botón para prueba
svst_button = tk.Button(root, text="Secret", command=Prueba.start)
svst_button.pack()
# Botón para go North
N_button = tk.Button(root, text="North", command=goNorth)
N_button.pack()
# Botón para go South
S_button = tk.Button(root, text="South", command=goSouth)
S_button.pack()
# Botón para go East
E_button = tk.Button(root, text="East", command=goEast)
E_button.pack()
# Botón para go West
W_button = tk.Button(root, text="West", command=goWest)
W_button.pack()
# Botón para RTL
RTL_button = tk.Button(root, text="RTL", command=RTL)
RTL_button.pack()
# Botón para armar drone
arm_button = tk.Button(root, text="Arm Drone", command=arm_and_takeoff)
arm_button.pack()
# Botón para cerrar ventana
close_button = tk.Button(root, text="Close", command=exit_simulation)
close_button.pack()

update = threading.Thread(target=update_info)
update.start()

root.mainloop()

print("Disconnected")
