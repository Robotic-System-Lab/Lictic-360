# Semantic Mapping Project
Project ini mengintegrasikan tiga komponen utama:

- gmapper: Bertugas melakukan pemetaan.
- ros_deep_learning: Berfungsi untuk launching kamera dan utilitas pendukung lainnya.
- segnet: Melakukan segmentasi dan deteksi objek untuk memberikan semantisasi pada pemetaan sehingga menjadi semantic mapping.

## Struktur Proyek
- !cmaker_ins: Folder ini merupakan requirement yang harus diinstall menggunakan CMake.
- gmapper: Package yang menyediakan fungsi pemetaan.
ros_deep_learning: Package yang menangani launching kamera dan utilitas terkait.
- segnet: Package untuk segmentasi & deteksi objek.