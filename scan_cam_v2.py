import time
import sys

import serial

import numpy as np
import cv2

args = sys.argv

pixels = 3694
exp = 20

com = None
preparing = False

offset = 30
# ガンマ変換用の数値準備
gamma = 3  # γ値を指定
img2gamma = np.zeros((256, 1), dtype=np.uint8)  # ガンマ変換初期値

# 公式適用
for i in range(256):
    img2gamma[i][0] = 255 * (float(i) / 255) ** (1.0 / gamma)

try:
    com = serial.Serial(args[1], baudrate=115200, parity=serial.PARITY_NONE, timeout=1)
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

com.write(b'\x03\x00')

while True:
    recv = com.read_until(b'\xff\xff\xff\xff')

    if len(recv) != 0:

        if len(recv) != pixels + 4:
            print("invalid data length : {}".format(len(recv)))
            continue

        preparing = False

        line = np.frombuffer(recv[:-4], dtype=np.uint8)
        line = 255 - line

        line[line <= offset] = 0
        line[line > offset] = line[line > offset] - offset
        line = line * (255 / (255 - offset))

        # line[line < bl] = 0
        # line = (line - bl) * v
        # line = 255 * np.power(line / 255, 1 / gamma)

        frame = np.roll(frame, (0, 1), axis=(0, 1))
        frame[:, 0] = line
    else:
        preparing = True
        time.sleep(1)

    # prev_img = cv2.LUT(frame, img2gamma)
    prev_img = cv2.resize(frame, None, fx=0.2, fy=0.2)

    zoom_img = frame[int(pixels / 2) - 100:int(pixels / 2) + 100, :200]
    zoom_img = cv2.resize(zoom_img, (500, 500))

    color = cv2.cvtColor(prev_img, cv2.COLOR_GRAY2BGR)
    cv2.putText(color, "Exposure : " + str(exp) + "ms", (0, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 3, cv2.LINE_AA)
    if preparing:
        cv2.putText(color, "Preparing...", (0, 100), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 3,
                    cv2.LINE_AA)

    cv2.imshow("preview", color)
    cv2.imshow("zoom", zoom_img)

    ret = cv2.waitKey()
    if ret == ord("q"):
        com.write(b'\x02\x00')
        break
    if ret == ord("w"):
        exp += 1
        com.write(b'\x01' + exp.to_bytes(1, 'big'))

    if ret == ord("e"):
        exp -= 1
        com.write(b'\x01' + exp.to_bytes(1, 'big'))

    if ret == ord("s"):
        com.write(b'\x03\x00')

    if ret == ord("i"):
        cv2.imwrite("out_{}.jpg".format(time.time()), frame)

# frame = cv2.LUT(frame, img2gamma)
cv2.imwrite("out_{}.jpg".format(time.time()), frame)
com.close()
