from flask import Flask, request, jsonify
import base64
import numpy as np
import cv2
import torch

app = Flask(__name__)
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
model = torch.hub.load('ultralytics/yolov5', 'custom', path="best.pt")

print(f"Conf: {model.conf}, IOU: {model.iou}")

model.conf = 0.5
# model.iou = 0.6

valid_objects = [
        "master_chef_can",
        "cracker_box",
        "sugar_box",
        "tomato_soup_can",
        "mustard_bottle",
        "tuna_fish_can",
        "pudding_box",
        "gelatin_box",
        "potted_meat_can",
        "banana",
        "strawberry",
        "apple",
        "lemon",
        "peach",
        "pear",
        "orange",
        "plum"
]

@app.post("/detect_objs")
def classify():
    if request.is_json:
        detection_request = request.get_json()
        b64img = detection_request["img"]
        w = detection_request["w"]
        h = detection_request["h"]
        c = detection_request["c"]

        b64img_decoded = base64.b64decode(b64img)
        # base64.de
        img = np.frombuffer(b64img_decoded, dtype=np.uint8)
        img = img.reshape((h, w, c))

        results = model(img)
        # print(type(results.pred))
        # print(results.pred)
        # results.pred = [ [p for p in results.pred[0] if model.names[int(p[5])] in valid_objects] ]
        object_names = [model.names[int(result[5])] for result in results.pred[0]]
        # print(object_names)
        obj_pos = {}
        result_list = results.pred[0].tolist()
        new_pred = []
        for name, result in zip(object_names, result_list):
            if name in valid_objects:
                new_pred.append(result)
                if name not in obj_pos:
                    obj_pos[name] = [result[:-1]]
                else:
                    obj_pos[name].append(result[:-1])
        # results.pred = torch.tensor(new_pred, device=results.pred[0].get_device())
        results.show()

        return obj_pos, 201
    return {"error": "Request must be JSON"}, 415
