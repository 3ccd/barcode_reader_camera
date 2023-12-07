import serial

import numpy as np
import cv2

pixels = 3694

com = None

try:
    com = serial.Serial("COM9", baudrate=115200, parity=serial.PARITY_NONE)
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


frame = np.zeros((pixels, 6000), dtype=np.uint8)

com.flush()

print("start receiving")

while True:
    recv = com.read_until(b'\xff\xff\xff\xff')
    if len(recv) != pixels + 4:
        print("invalid data")
        continue

    line = np.frombuffer(recv[:-4], dtype=np.uint8)
    line = 255 - line

    # line[line < bl] = 0
    # line = (line - bl) * v
    # line = 255 * np.power(line / 255, 1 / gamma)

    frame = np.roll(frame, (0, 1), axis=(0, 1))
    frame[:, 0] = line

    prev_img = cv2.resize(frame, None, fx=0.2, fy=0.2)

    cv2.imshow("preview", prev_img)

    ret = cv2.waitKey(1)
    if ret == ord("q"):
        break

com.close()
