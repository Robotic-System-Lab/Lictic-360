import torch

# Load model YOLOv5 dari file .pt
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Atau varian lain seperti yolov5m, yolov5l

# Set model ke mode evaluasi
model.eval()

# Ekspor model ke ONNX
dummy_input = torch.randn(1, 3, 640, 640)  # Ukuran input yang diperlukan
torch.onnx.export(model, dummy_input, "yolov5s.onnx", opset_version=11)