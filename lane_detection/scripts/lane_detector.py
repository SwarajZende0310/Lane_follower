import cv2
import numpy as np
import math

def mask_yellow(frame):
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

  lower_yellow = np.array([26,76,180])
  upper_yellow = np.array([91,255,255])

  mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

  masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

  return masked_frame


def main():

    cap = cv2.VideoCapture(15)
    
    theta=0
    minLineLength = 5
    maxLineGap = 10

    while True:
        ret,frame = cap.read()

        image = frame
        image = cv2.resize(image,(500,300))
        gray = mask_yellow(frame)
        cv2.imshow('gray',gray)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edged = cv2.Canny(blurred, 85, 85)
        lines = cv2.HoughLinesP(edged,1,np.pi/180,10,minLineLength,maxLineGap)
        if lines is not None:
            for x in range(0, len(lines)):
                for x1,y1,x2,y2 in lines[x]:
                    cv2.line(image,(x1,y1),(x2,y2),(0,255,0),2)
                    theta=theta+math.atan2((y2-y1),(x2-x1))
                    print(theta)

        threshold=6
        if(theta>threshold):
            print("left")
        if(theta<-threshold):
            print("right")
        if(abs(theta)<threshold):
            print("straight")

        theta = 0

        cv2.imshow('frame',frame)
        if cv2.waitKey(1)==ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()