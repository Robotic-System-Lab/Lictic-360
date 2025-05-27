import onnxruntime as ort
import numpy as np

# Muat model YOLOv5 ONNX
ort_session = ort.InferenceSession("yolov5s.onnx")

# Periksa input dan output model
input_name = ort_session.get_inputs()[0].name
output_name = ort_session.get_outputs()[0].name
print(f"Input Name: {input_name}")
print(f"Output Name: {output_name}")

# Buat input dummy yang sesuai dengan ukuran input model YOLOv5 (misalnya 1x3x640x640)
dummy_input = np.random.random((1, 3, 640, 640)).astype(np.float32)

# Lakukan inferensi
result = ort_session.run([output_name], {input_name: dummy_input})

# Tampilkan hasil inferensi
print("Hasil inferensi:", result)
