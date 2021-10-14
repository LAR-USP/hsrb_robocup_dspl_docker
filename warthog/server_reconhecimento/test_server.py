#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import requests
import base64
import ast
import cv2

img = cv2.imread('sim.png')
img = cv2.resize(img, (640, 480))

def request_classification(img):
    url = 'http://172.19.0.1:5000/detect_objs'
    # retval, imbuffer = cv2.imencode('.png', img)
    myobj = {
        "img": str(base64.b64encode(img.ravel().tobytes())),
        "h": img.shape[0],
        "w": img.shape[1],
        "c": img.shape[2]
    }
    return ast.literal_eval(requests.post(url, json= myobj).text)

request_classification(img)
