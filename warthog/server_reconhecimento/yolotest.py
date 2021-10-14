from sys import path
import torch

# Model
# model = torch.hub.load('archive', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5m, yolov5l, yolov5x, custom
model = torch.hub.load('ultralytics/yolov5', 'custom', path="best.pt")
# model.load_state_dict(torch.load('best.pt')['model'].state_dict())

# Images
img = 'https://ultralytics.com/images/zidane.jpg'  # or file, Path, PIL, OpenCV, numpy, list

# Inference
results = model(img)

# Results
print(type(results))
for result in results.pred[0]:
    print(f'Top left: ({result[0]}, {result[1]}). Bottom right: ({result[2]}, {result[3]}). Confidence: {result[4]}. Class: {model.names[int(result[5])]}')
results.print()  # or .show(), .save(), .crop(), .pandas(), etc.
results.show()