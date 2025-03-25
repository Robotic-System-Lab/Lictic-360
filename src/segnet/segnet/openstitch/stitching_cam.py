from stitching import Stitcher
import cv2

# Tentukan gambar yang akan digabungkan
images = ["img1.jpg", "img2.jpg", "img3.jpg"]  # Ganti dengan nama file gambar Anda

# Membuat objek Stitcher dengan pengaturan detektor dan ambang batas kepercayaan yang lebih rendah
stitcher = Stitcher(detector="sift", confidence_threshold=0.1)

# Menjalankan proses stitching
panorama = stitcher.stitch(images)

# Menyimpan hasil panorama
cv2.imwrite("panorama2.jpg", panorama)
