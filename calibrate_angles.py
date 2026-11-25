from machine import PWM, Pin
from servo_translator import translate
import time

shoulder = PWM(Pin(0))
elbow = PWM(Pin(1))
pen = PWM(Pin(2))
shoulder.freq(50)
elbow.freq(50)
pen.freq(50)

PEN_UP = 0
PEN_DOWN = 30


# draws a dot at every 10 degrees from 0 to 180
def mark_sweep(servo, name):
    for angle in range(0, 190, 10):
        pen.duty_u16(translate(PEN_UP))       # lift before moving, no drag lines
        servo.duty_u16(translate(angle))
        time.sleep(1)                         # wait for the arm to get there
        print(name, angle)                    # so we know which dot is which
        pen.duty_u16(translate(PEN_DOWN))
        time.sleep(1)
    pen.duty_u16(translate(PEN_UP))


# shoulder sweep: elbow held so the arm is straight
elbow.duty_u16(translate(135))
mark_sweep(shoulder, "shoulder")

input("Shoulder done. Change the paper and press Enter for the elbow sweep")

# elbow sweep: shoulder held still
shoulder.duty_u16(translate(130))
mark_sweep(elbow, "elbow")
print("Done. Measure the dots and enter them with save_data.py")
