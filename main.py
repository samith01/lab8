from machine import PWM, Pin
from servo_translator import translate
from math import sqrt, acos, asin, sin, degrees
import time

# arm lengths and shoulder position (mm)
LA = 155.0
LB = 155.0
AX = -50.0    # shoulder is 50 mm left of the paper
AY = 139.5    # shoulder is centered on the paper height

# offsets from eq. 8 and 9, tune these on the jig
SHOULDER_OFFSET = 75
ELBOW_OFFSET = 150

PEN_UP = 0
PEN_DOWN = 30

shoulder = PWM(Pin(0))
elbow = PWM(Pin(1))
pen = PWM(Pin(2))
shoulder.freq(50)
elbow.freq(50)
pen.freq(50)


# converts a pen position (x, y) into the two servo angles
def inverse_kinematics(cx, cy):
    ac = sqrt((AX - cx) ** 2 + (AY - cy) ** 2)      # shoulder to pen distance
    abase_c = sqrt((AX - cx) ** 2 + cy ** 2)        # A_base (below shoulder) to pen
    bac = acos((LA ** 2 + ac ** 2 - LB ** 2) / (2 * LA * ac))  # law of cosines, angle at shoulder
    acb = asin(LA * sin(bac) / LB)                  # law of sines, angle at the pen
    yac = acos((AY ** 2 + ac ** 2 - abase_c ** 2) / (2 * AY * ac))  # angle from the vertical to the pen
    alpha = degrees(bac + yac)   # full shoulder angle measured from the vertical
    beta = degrees(bac + acb)    # outside angle at the elbow
    # offsets turn these into the angles the servos expect
    return alpha - SHOULDER_OFFSET, ELBOW_OFFSET - beta


# reads lines like "shoulder 10 16" (servo, angle sent, angle measured)
# and stores (angle, error) for each servo
def calibration_setup(filename):
    table = {"shoulder": [], "elbow": []}
    f = open(filename)
    for line in f:
        parts = line.split()
        if len(parts) == 3 and parts[0] in table:
            sent = float(parts[1])
            measured = float(parts[2])
            table[parts[0]].append((sent, measured - sent))   # error = how far off the servo was
    f.close()
    # sort by angle so get_error can walk through them in order
    table["shoulder"].sort()
    table["elbow"].sort()
    return table


# finds the error at any angle by drawing a line between the two closest points
def get_error(points, angle):
    if len(points) == 0:
        return 0                 # no data, so no correction
    if angle <= points[0][0]:
        return points[0][1]      # below the first point, use its error
    for i in range(len(points) - 1):
        a0, e0 = points[i]
        a1, e1 = points[i + 1]
        if angle <= a1:
            # angle is between a0 and a1, take the error part way between e0 and e1
            return e0 + (e1 - e0) * (angle - a0) / (a1 - a0)
    return points[-1][1]         # past the last point, use its error


# sends the two angles to the servos, corrected with the error table
def send_compensated_angle(table, a, b, use_cal=True):
    # the servo overshoots by the error, so subtract it
    if use_cal:
        a = a - get_error(table["shoulder"], a)
        b = b - get_error(table["elbow"], b)
    shoulder.duty_u16(translate(a))
    elbow.duty_u16(translate(b))


def move_arm(table, x, y, use_cal):
    a, b = inverse_kinematics(x, y)
    send_compensated_angle(table, a, b, use_cal)


def draw_square(table, use_cal):
    corners = [(50, 80), (150, 80), (150, 180), (50, 180), (50, 80)]

    pen.duty_u16(translate(PEN_UP))
    move_arm(table, 50, 80, use_cal)
    time.sleep(1)                     # give the arm time to reach the first corner
    pen.duty_u16(translate(PEN_DOWN))

    # split each side into 50 small steps so the line comes out straight
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        for step in range(1, 51):
            t = step / 50   # how far along the side we are, 0 to 1
            move_arm(table, x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, use_cal)
            time.sleep(0.02)   # small pause so the servos keep up

    pen.duty_u16(translate(PEN_UP))


table = calibration_setup("calibration_data_5.txt")
draw_square(table, False)   # without calibration
time.sleep(2)
draw_square(table, True)    # with calibration, compare the two squares
