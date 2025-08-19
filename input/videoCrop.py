import cv2
import numpy as np

cap = cv2.VideoCapture("20210905_3rd_01(255Hz).mp4")

ret, frame = cap.read()

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
obj_out = cv2.VideoWriter('512x256_20210905_3rd_01(255Hz).mp4',fourcc, 30.0, (512, 256))


while (ret != False):
    obj_out.write(frame[:,:512-640,:])

    ret, frame = cap.read()

obj_out.release()
cap.release()