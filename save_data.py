# Type in the distances measured on the paper, saves them to calibration_data_5.txt
# Each saved line: <servo> <angle sent> <actual angle>
# The actual angle is measured from the 0 deg dot, like a protractor zeroed on it
from math import acos, asin, degrees

L = 137.0       # each arm link, pen is 274 mm from the shoulder when straight
REF = 135.0     # shoulder pivot to the reference point (A_base), mm


# law of cosines: angle between sides a and b, given the side across from it
def angle(a, b, opposite):
    return degrees(acos((a * a + b * b - opposite * opposite) / (2 * a * b)))


servo = input("Which sweep? shoulder / elbow: ").strip()

# read all 19 distances first, we need them all to know where the dots switch sides
d = []
for sent in range(0, 190, 10):
    d.append(float(input(str(sent) + " deg distance (mm): ")))

# the closest dot is where the arm passes the reference point,
# dots after it are on the other side so their angle is negative
closest = d.index(min(d))

angles = []
for i in range(len(d)):
    if servo == "shoulder":
        # arm is straight, so shoulder -> dot is 2 * L
        a = angle(REF, 2 * L, d[i])
    else:
        # distance is shoulder -> dot, both links are L, gives the inside elbow angle
        a = 2 * degrees(asin(d[i] / (2 * L)))
    if servo == "shoulder" and i < closest:
        a = -a        # shoulder sweeps toward the reference point first
    if servo == "elbow" and i >= closest:
        a = -a        # pen has folded past the shoulder
    angles.append(a)

f = open("calibration_data_5.txt", "a")   # "a" so old data isn't erased
for i in range(len(angles)):
    # shoulder angle grows with the servo, elbow inside angle shrinks
    if servo == "shoulder":
        actual = angles[i] - angles[0]
    else:
        actual = angles[0] - angles[i]
    print(i * 10, "deg  actual =", round(actual, 1))
    f.write(servo + " " + str(i * 10) + " " + str(round(actual, 1)) + "\n")
f.close()
print("Saved to calibration_data_5.txt")
