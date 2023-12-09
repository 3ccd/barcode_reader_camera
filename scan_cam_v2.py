import time

import serial

import numpy as np
import cv2

pixels = 3694

com = None

offset = 30
# ガンマ変換用の数値準備
gamma = 3  # γ値を指定
img2gamma = np.zeros((256, 1), dtype=np.uint8)  # ガンマ変換初期値

# 公式適用
for i in range(256):
    img2gamma[i][0] = 255 * (float(i) / 255) ** (1.0 / gamma)

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
        print("invalid data length : {}".format(len(recv)))
        continue

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

    prev_img = cv2.LUT(frame, img2gamma)
    prev_img = cv2.resize(prev_img, None, fx=0.2, fy=0.2)

    zoom_img = frame[int(pixels / 2) - 100:int(pixels / 2) + 100, :200]
    zoom_img = cv2.resize(zoom_img, (500, 500))

    cv2.imshow("preview", prev_img)
    cv2.imshow("zoom", zoom_img)

    ret = cv2.waitKey(1)
    if ret == ord("q"):
        break

frame = cv2.LUT(frame, img2gamma)
cv2.imwrite("out_{}.jpg".format(time.time()), frame)
com.close()
