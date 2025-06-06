import cv2
import numpy as np

# Fungsi untuk koreksi white balance
def apply_white_balance(image):
    result = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(result)
    l = cv2.equalizeHist(l)
    result = cv2.merge((l, a, b))
    result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
    return result

# Fungsi untuk menyesuaikan kecerahan dan kontras
def adjust_brightness_contrast(image, brightness=0, contrast=30):
    beta = brightness
    alpha = contrast / 100.0 + 1.0
    result = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return result

# Fungsi untuk koreksi distorsi lensa
def undistort_image(image, camera_matrix, dist_coeffs):
    h, w = image.shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted_image

# Pipeline GStreamer untuk mengakses kamera CSI
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )

# Parameter kamera (ganti dengan hasil kalibrasi Anda)
camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)
dist_coeffs = np.array([0.1, -0.25, 0, 0, 0], dtype=np.float32)

# Membuka kamera CSI
cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Tidak dapat membuka kamera CSI.")
    exit()

print("Tekan 'q' untuk keluar.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Gagal membaca frame.")
        break

    # Koreksi white balance
    frame = apply_white_balance(frame)

    # Koreksi distorsi lensa
    frame = undistort_image(frame, camera_matrix, dist_coeffs)

    # Penyesuaian kecerahan dan kontras
    frame = adjust_brightness_contrast(frame, brightness=10, contrast=50)

    # Tampilkan frame hasil pemrosesan
    cv2.imshow("Hasil Kamera CSI", frame)

    # Tekan 'q' untuk keluar dari loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Bersihkan sumber daya
cap.release()
cv2.destroyAllWindows()
