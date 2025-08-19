import cv2
import numpy as np

cap = cv2.VideoCapture("512x256_20210905_3rd_01(255Hz).mp4")

ret, frame = cap.read()

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
obj_out = cv2.VideoWriter('32x16_20210905_3rd_01(255Hz).mp4',fourcc, 30.0, (32, 16))


while (ret != False):
    obj_out.write(
        cv2.resize(frame, (0, 0), fx=0.0625, fy=0.0625, interpolation=cv2.INTER_AREA)
    )

    ret, frame = cap.read()

obj_out.release()
cap.release()
