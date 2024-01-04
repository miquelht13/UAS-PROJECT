import cv2 as cv
import paho.mqtt.client as mqtt
import base64
import time
from dronekit import connect, VehicleMode
import threading
import numpy as np
from pymavlink import mavutil
import math

broker_address = "localhost"
broker_port = 1883


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
    #time.sleep(1)


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


def update_info():
    global updating
    status = ""
    mode = "Waiting to arm..."
    altitude = "Waiting to arm..."
    heading = "Waiting to arm..."
    while updating:
        if not vehicle.armed:
            status = "Not armed"
        if vehicle.armed:
            status = "Armed"
            mode = str(vehicle.mode.name)
            altitude = str(vehicle.location.global_relative_frame.alt)
            heading = str(vehicle.heading)
        if vehicle.mode.name == 'RTL':
            if not vehicle.armed:
                status = "Landed"

        client.publish('status', status)
        client.publish('mode', mode)
        client.publish('altitude', altitude)
        client.publish('heading', heading)
        time.sleep(1)


def arm_and_takeoff():
    status = "Basic Pre-Arm checks"

    while not vehicle.is_armable:
        status = "Basic Pre-Arm checks"
        time.sleep(1)

    status = "Arming motors"
    # Drone should be armed in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed
    while not vehicle.armed:
        status = "Waiting for arming"
        time.sleep(1)

    status = "Taking off!"
    client.publish('status', status)
    vehicle.simple_takeoff(6)
    time.sleep(5)
    status = "Ready to detect colors and moving"
    client.publish('status', status)


def detectColor():
    global detectingColor
    global isRed
    global isBlue
    global isYellow
    global isPink
    color = "None"
    cap = cv.VideoCapture(0)
    isRed = False
    isBlue = False
    isYellow = False
    isPink = False
    while detectingColor:
        ret, frame = cap.read()
        _, buffer = cv.imencode('.jpg', frame)
        # Converting into encoded bytes for transmission
        jpg_as_txt = base64.b64encode(buffer)
        client.publish('picture', jpg_as_txt)

        img = frame[yI:yF, xI:xF, :]
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # define range of red color in HSV
        lower_red = np.array([0, 100, 100])  # min HSV values of red
        upper_red = np.array([20, 255, 255])  # max HSV values of red
        # Threshold the HSV image to get only red colors
        mask_red = cv.inRange(hsv, lower_red, upper_red)
        if mask_red.mean() > 50:
            print("RED")
            isRed = True
            isBlue = False
            isYellow = False
            isPink = False

        # define range of blue color in HSV
        lower_blue = np.array([85, 100, 100])  # min HSV values of blue
        upper_blue = np.array([100, 255, 255])  # max HSV values of blue
        # Threshold the HSV image to get only blue colors
        mask_blue = cv.inRange(hsv, lower_blue, upper_blue)
        if mask_blue.mean() > 50:
            print("BLUE")
            isRed = False
            isBlue = True
            isYellow = False
            isPink = False

        # define range of yellow color in HSV
        lower_yellow = np.array([25, 100, 100])  # min HSV values of yellow
        upper_yellow = np.array([35, 255, 255])  # max HSV values of yellow
        # Threshold the HSV image to get only yellow colors
        mask_yellow = cv.inRange(hsv, lower_yellow, upper_yellow)
        if mask_yellow.mean() > 50:
            print("YELLOW")
            isRed = False
            isBlue = False
            isYellow = True
            isPink = False

        # define range of pink color in HSV
        lower_pink = np.array([150, 100, 100])  # min HSV values of pink
        upper_pink = np.array([165, 255, 255])  # max HSV values of pink
        # Threshold the HSV image to get only yellow colors
        mask_pink = cv.inRange(hsv, lower_pink, upper_pink)
        if mask_pink.mean() > 50:
            print("PINK")
            isRed = False
            isBlue = False
            isYellow = False
            isPink = True
            detectingColor = False

        else:
            print("No color")

        time.sleep(0.25)


def DistanceInMeters(loc1, loc2):
    dlat = loc1.lat - loc2.lat
    dlon = loc1.lon - loc2.lon
    return math.sqrt((dlat*dlat)+(dlon*dlon))*1.113195e5


def RTL():
    vehicle.mode = VehicleMode("RTL")


def startMoving():
    global moving
    n = 0
    checkpoints = [0, 0, 0, 0, 0]
    r = 0
    b = 0
    y = 0
    p = 0
    while moving:
        if isRed:
            b = 0
            y = 0
            p = 0
            if r == 0:
                r = r + 1
                checkpoints[n] = vehicle.location.global_frame
                n = n + 1
            if not r == 0:
                r = r + 1

            goNorth()

        if isBlue:
            r = 0
            y = 0
            p = 0
            if b == 0:
                b = b + 1
                checkpoints[n] = vehicle.location.global_frame
                n = n + 1
            if not b == 0:
                b = b + 1

            goWest()

        if isYellow:
            r = 0
            b = 0
            p = 0
            if y == 0:
                y = y + 1
                checkpoints[n] = vehicle.location.global_frame
                n = n + 1
            if not y == 0:
                y = y + 1

            goEast()

        if isPink:
            # goEast()
            moving = False
            break

        time.sleep(0.5)

    time.sleep(2)
    # Baja el dron
    checkpointEntrega = vehicle.location.global_relative_frame
    checkpointEntrega.alt = 3
    vehicle.simple_goto(checkpointEntrega)
    time.sleep(6)
    # Hacer acción servo
    time.sleep(2)
    move_servo(servo_channel, servo_pwm_value)
    time.sleep(2)
    # Sube el dron
    checkpointEntrega = vehicle.location.global_relative_frame
    checkpointEntrega.alt = 6
    vehicle.simple_goto(checkpointEntrega)
    time.sleep(6)

    # Vuelta checkpoint
    time.sleep(1)
    i = 0
    while i < n:  # Asi no va al primer punto (el rojo de takeoff) y al anterior hace RTL
        vehicle.simple_goto(checkpoints[n - i])
        currentlocaton = vehicle.location.global_frame
        dist = DistanceInMeters(checkpoints[n - i], currentlocaton)
        # Wait until reach point
        while dist > 0.5:
            time.sleep(0.25)
            currentlocaton = vehicle.location.global_frame
            dist = DistanceInMeters(checkpoints[n - i], currentlocaton)
        time.sleep(1)
        i = i + 1

    RTL()


# Función para mover el servo
def move_servo(channel, pwm_value):
    # Enviar comando PWM al servo a través del canal específico
    vehicle.channels.overrides[channel] = pwm_value


def prueba_move_servo():
    # Prueba para mover el servo a su mínimo y máximo
    vehicle.channels.overrides[8] = 1900
    time.sleep(5)
    vehicle.channels.overrides[8] = 1100


def on_message(client, userdata, message):
    global updating
    global detectingColor
    global moving

    if message.topic == 'updateInfo':
        print('update info')
        updating = True
        update = threading.Thread(target=update_info)
        update.start()

    if message.topic == 'arm&takeoff':
        print('arm and takeoff')
        arm_and_takeoff()

    if message.topic == 'startDetectColor':
        print('start detect color')
        detectingColor = True
        color = threading.Thread(target=detectColor)
        color.start()

    if message.topic == 'stopDetectColor':
        print('stop detect color')
        detectingColor = False

    if message.topic == 'startMoving':
        print('start moving')
        moving = True
        move = threading.Thread(target=startMoving)
        move.start()

    if message.topic == 'stopMoving':
        print('start moving')
        moving = False

    if message.topic == 'servoPrueba':
        print('probando servo')
        prueba_move_servo()


detectingColor = False
moving = False
updating = False

xI, yI, xF, yF = 295, 215, 355, 265

speed = 0.5

servo_channel = 8  # AUX OUT 3 generalmente se asigna al canal 11
servo_pwm_value = 1900

connection_string = "/dev/ttyS0"
vehicle = connect(connection_string, wait_ready=True, baud=115200)

client = mqtt.Client("On board controller")
client.on_message = on_message
client.connect(broker_address, broker_port)
client.subscribe('updateInfo')
client.subscribe('arm&takeoff')
client.subscribe('startDetectColor')
client.subscribe('startMoving')
client.subscribe('stopDetectColor')
client.subscribe('stopMoving')
client.subscribe('servoPrueba')
client.loop_forever()
