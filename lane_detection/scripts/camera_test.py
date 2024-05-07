import cv2

def main():
    print('Camera_test')

    cap = cv2.VideoCapture(2)
    
    while True:
        ret,frame = cap.read()

        cv2.imshow('frame',frame)
        if cv2.waitKey(1)==ord('q'):
            break

if __name__ == '__main__':
    main()