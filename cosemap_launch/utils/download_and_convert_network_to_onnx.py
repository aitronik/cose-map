from ultralytics import YOLO
import subprocess

model = YOLO("../models/yolov8m-seg.pt")
path = model.export(format="onnx") 
