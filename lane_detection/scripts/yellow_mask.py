import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow("Trackbars")

cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 200, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - S", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

def mask_yellow(frame):
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  l_h = cv2.getTrackbarPos("L - H", "Trackbars")
  l_s = cv2.getTrackbarPos("L - S", "Trackbars")
  l_v = cv2.getTrackbarPos("L - V", "Trackbars")
  u_h = cv2.getTrackbarPos("U - H", "Trackbars")
  u_s = cv2.getTrackbarPos("U - S", "Trackbars")
  u_v = cv2.getTrackbarPos("U - V", "Trackbars")
#   lower_yellow = np.array([19,106,185])
#   upper_yellow = np.array([47,149,255])
  lower_yellow = np.array([l_h,l_s,l_v])
  upper_yellow = np.array([u_h,u_s,u_v])

  mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

  masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

  return masked_frame

# Capture video from webcam
cap = cv2.VideoCapture(15)

while True:
  # Read a frame from the webcam
  ret, frame = cap.read()

  # Apply yellow masking function
  masked_frame = mask_yellow(frame)

  # Display the masked frame
  cv2.imshow('Webcam Feed (Yellow Masked)', masked_frame)

  # Exit on 'q' press
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

# Release resources
cap.release()
cv2.destroyAllWindows()
