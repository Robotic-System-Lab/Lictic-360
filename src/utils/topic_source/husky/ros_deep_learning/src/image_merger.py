#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Sensor IDs, sesuaikan dengan topik gambar dari masing-masing kamera
CAMERA_TOPICS = [
    "/camera1/image_raw",  # Kamera 1
    "/camera2/image_raw",  # Kamera 2
    "/camera3/image_raw",  # Kamera 3
    "/camera4/image_raw",  # Kamera 4
    "/camera5/image_raw",  # Kamera 5
    "/camera6/image_raw"   # Kamera 6
]

class ImageMerger:
    def __init__(self):
        # Inisialisasi ROS node
        rospy.init_node('image_merger', anonymous=True)

        # Inisialisasi CvBridge untuk konversi pesan gambar ROS ke format OpenCV
        self.bridge = CvBridge()

        # Membuat subscriber untuk setiap kamera
        self.images = [None] * len(CAMERA_TOPICS)  # Menyimpan gambar dari setiap kamera
        for i, topic in enumerate(CAMERA_TOPICS):
            rospy.Subscriber(topic, Image, self.image_callback, callback_args=i)

        # Menyiapkan jendela untuk menampilkan gambar gabungan
        cv2.namedWindow("Gabungan Gambar 2x3", cv2.WINDOW_NORMAL)

    def image_callback(self, msg, camera_index):
        try:
            # Mengonversi gambar dari ROS ke OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Simpan gambar yang diterima ke dalam array images
            self.images[camera_index] = cv_image
            
            # Jika semua gambar sudah diterima, gabungkan gambar-gambar tersebut
            if all(img is not None for img in self.images):
                self.merge_and_show_images()

        except Exception as e:
            rospy.logerr(f"Error saat mengonversi gambar: {e}")

    def merge_and_show_images(self):
        # Gabungkan gambar dari 6 kamera (2 baris dan 3 kolom)
        row1 = np.hstack(self.images[:3])  # Gabungkan 3 gambar pertama (baris 1)
        row2 = np.hstack(self.images[3:])  # Gabungkan 3 gambar berikutnya (baris 2)
        combined_image = np.vstack([row1, row2])  # Gabungkan 2 baris menjadi 1 frame
        
        # Menampilkan gambar gabungan
        cv2.imshow("Gabungan Gambar 2x3", combined_image)

        # Menunggu untuk menutup jendela dengan tombol 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("Program dihentikan oleh pengguna.")

    def run(self):
        # Menjalankan ROS spin untuk menjaga node tetap berjalan
        rospy.spin()

        # Menutup semua jendela OpenCV saat selesai
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        # Membuat objek ImageMerger dan menjalankan
        image_merger = ImageMerger()
        image_merger.run()
    except rospy.ROSInterruptException:
        pass
