from ultralytics import YOLO

model = YOLO("runs/detect/drone_xml_model_full/weights/best.pt")
model.export(
    format="onnx",
    imgsz=640,
    opset=11,       # ← opset 11 instead of 12
    simplify=True,
    dynamic=False,
    nms=False,      # ← no NMS
)