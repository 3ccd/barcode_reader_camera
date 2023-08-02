import serial

import numpy as np
import cv2

pixels = 3694

com = None

try:
    com = serial.Serial("COM4", baudrate=115200, parity=serial.PARITY_NONE)
except serial.SerialException as e:
    print(e)
    exit(-1)


if com.is_open:
    com.close()

try:
    com.open()
except serial.SerialException as e:
    print(e)
    exit(-1)


frame = np.zeros((900, pixels), dtype=np.uint8)

bl = 80
gamma = 2
v = 255 / (255 - bl)
print(v)

com.flush()

print("start receiving")

while True:
    recv = com.read_until(b'\xff\xff')
    if len(recv) != pixels + 2:
        continue

    line = np.frombuffer(recv[:-2], dtype=np.uint8)
    line = 255 - line

    # line[line < bl] = 0
    # line = (line - bl) * v
    # line = 255 * np.power(line / 255, 1 / gamma)

    frame = np.roll(frame, (0, 1), axis=(1, 0))
    frame[0, :] = line

    prev_img = cv2.resize(frame, None, fx=0.5, fy=1)

    cv2.imshow("preview", prev_img)

    ret = cv2.waitKey(1)
    if ret == ord("q"):
        break

com.close()
