import cv2

# Opens the Video file
cap= cv2.VideoCapture('/home/pcarboni/TESP/TESP2024/rocks_sample_images/video_rocks.mp4')
i=0
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == False:
        break
    if i % 50 == 0: # this is the line I added to make it only save one frame every 20
        cv2.imwrite('rock_frame_'+str(i)+'.jpg',frame)
    i+=1

cap.release()
cv2.destroyAllWindows()