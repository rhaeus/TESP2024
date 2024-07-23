from ultralytics import YOLO

print('hello')
model = YOLO("yolov8m.pt")

print('results')
results = model.predict("/home/pcarboni/TESP/TESP2024/rocks_sample_images/rocks1.jpeg")

print('result[0]')
result = results[0]

len = len(result.boxes) # how many bounding boxes were found
print("Number of detected objects", len)


box = result.boxes[0] #the first detected box


print("Coordinates of box 1:", box.xyxy) #top left and bottom right coordinates of the box: [x_tl, y_tl, x_br, y_br]
print("Probability of the detection:", box.conf) #confidence level about the detected object




