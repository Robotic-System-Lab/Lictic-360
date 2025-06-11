import os
import sys
import glob
import cv2
import tempfile
import shutil

import rclpy
from rclpy.node import Node

from PySide6.QtWidgets import (
  QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
  QPushButton, QFileDialog, QMessageBox, QDialog, QListWidget, QDialogButtonBox
)
from PySide6.QtMultimedia import QMediaPlayer
from PySide6.QtMultimediaWidgets import QVideoWidget
from PySide6.QtCore import QUrl, QTimer
import time

class SortedFolderDialog(QDialog):
  def __init__(self, default_folder):
    super().__init__()
    self.setWindowTitle("Pilih Folder dengan .png")
    layout = QVBoxLayout(self)
    self.list_widget = QListWidget()
    # Tambahkan padding dan background color untuk item, serta warna berbeda untuk item terpilih
    self.list_widget.setStyleSheet("""
      QListWidget::item { 
        padding: 10px; 
      }
      QListWidget::item:selected { 
        background-color: #30A2C8;
        border: none;
        outline: none;
      }
    """)
    # Tangani double click pada list item untuk memilih folder langsung
    self.list_widget.itemDoubleClicked.connect(lambda item: self.accept())
    layout.addWidget(self.list_widget)

    # Ambil semua folder dalam default_folder dan sort berdasarkan waktu modifikasi (desc)
    dirs = [os.path.join(default_folder, d) for d in os.listdir(default_folder)
            if os.path.isdir(os.path.join(default_folder, d))]
    dirs.sort(key=lambda d: os.path.getmtime(d), reverse=True)

    # Jika tidak ada subfolder, tambahkan default_folder itu sendiri
    if not dirs:
      dirs = [default_folder]

    for d in dirs:
      self.list_widget.addItem(d)

    button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
    button_box.accepted.connect(self.accept)
    button_box.rejected.connect(self.reject)
    layout.addWidget(button_box)

  def selectedFolder(self):
    item = self.list_widget.currentItem()
    if item:
      return item.text()
    return None

class VideoGeneratorNode(Node):
  def __init__(self):
    super().__init__('qtvid')
    self.get_logger().info('Video Generator Node started.')


class VideoGeneratorGUI(QMainWindow):
  def __init__(self, node: VideoGeneratorNode):
    super().__init__()
    self.node = node
    self.folder_path = ""
    self.temp_video_path = ""
    self.setWindowTitle("Video Generator")
    self.resize(820, 850)  # ukuran window agar nampak preview 800x800 + tombol

    # Widget utama
    central_widget = QWidget()
    self.setCentralWidget(central_widget)
    layout = QVBoxLayout(central_widget)

    # QVideoWidget untuk preview video
    self.video_widget = QVideoWidget()
    self.video_widget.setFixedSize(800, 800)
    layout.addWidget(self.video_widget)

    # QMediaPlayer untuk memutar video
    self.player = QMediaPlayer()
    self.player.setVideoOutput(self.video_widget)
    self.player.mediaStatusChanged.connect(self.on_media_status_changed)

    # Layout untuk tombol
    btn_layout = QHBoxLayout()
    layout.addLayout(btn_layout)

    self.btn_choose = QPushButton("Pilih Folder")
    self.btn_choose.clicked.connect(self.choose_folder)
    btn_layout.addWidget(self.btn_choose)

    self.btn_start_stop = QPushButton("Start video")
    self.btn_start_stop.clicked.connect(self.toggle_video_start_stop)
    btn_layout.addWidget(self.btn_start_stop)

    self.btn_pause_resume = QPushButton("Pause video")
    self.btn_pause_resume.clicked.connect(self.toggle_pause_resume_video)
    btn_layout.addWidget(self.btn_pause_resume)

    self.btn_save = QPushButton("Save")
    self.btn_save.clicked.connect(self.save_video)
    btn_layout.addWidget(self.btn_save)

  def choose_folder(self):
    default_folder = os.path.expanduser("~/lictic/captured_map")
    if not os.path.exists(default_folder):
      os.makedirs(default_folder, exist_ok=True)

    dialog = SortedFolderDialog(default_folder)
    if dialog.exec() == QDialog.Accepted:
      folder = dialog.selectedFolder()
      if folder:
        self.player.stop()
        self.temp_video_path = ""
        self.folder_path = folder
        self.node.get_logger().info(f"Folder dipilih: {self.folder_path}")
        QMessageBox.information(self, "Folder Dipilih", f"Folder: {self.folder_path}")

  def toggle_video_start_stop(self):
    # Jika video sedang diputar, maka stop video
    if self.player.playbackState() == QMediaPlayer.PlayingState:
      self.player.stop()
      self.btn_start_stop.setText("Start video")
      self.btn_pause_resume.setText("Pause video")
    else:
      # Jika belum ada video, generate dulu
      if self.folder_path == "":
        QMessageBox.warning(self, "Error", "Silakan pilih folder terlebih dahulu.")
        return
      
      # Jika video belum digenerate, ubah tombol ke "Processing" dan tunggu 1 detik sebelum generate
      if not self.temp_video_path and self.folder_path:
        self.btn_start_stop.setText("Processing")
        QTimer.singleShot(1000, self.process_video_generation)
        return
      
      # Jika video sudah tersedia, langsung putar
      self.player.play()
      self.btn_start_stop.setText("Stop video")
      self.btn_pause_resume.setText("Pause video")
  
  def process_video_generation(self):
    video_path = self.generate_video(self.folder_path)
    if video_path:
      self.temp_video_path = video_path
      url = QUrl.fromLocalFile(video_path)
      self.player.setSource(url)
    self.player.play()
    self.btn_start_stop.setText("Stop video")
    self.btn_pause_resume.setText("Pause video")
  
  def toggle_pause_resume_video(self):
    # Jika video sedang diputar, pause video
    if self.player.playbackState() == QMediaPlayer.PlayingState:
      self.player.pause()
      self.btn_start_stop.setText("Start video")
      self.btn_pause_resume.setText("Resume video")
    # Jika video sedang dipause, resume video
    elif self.player.playbackState() == QMediaPlayer.PausedState:
      self.player.play()
      self.btn_pause_resume.setText("Pause video")
    else:
      QMessageBox.warning(self, "Error", "Video belum dimulai.")

  def save_video(self):
    if not self.temp_video_path or not os.path.exists(self.temp_video_path):
      QMessageBox.warning(self, "Error", "Tidak ada video untuk disimpan, silakan mulai video terlebih dahulu.")
      return

    filename, _ = QFileDialog.getSaveFileName(self, "Simpan Video", "", "MP4 files (*.mp4)")
    if filename:
      try:
        shutil.copy(self.temp_video_path, filename + ".mp4" if not filename.endswith('.mp4') else filename)
        self.node.get_logger().info(f"Video berhasil disimpan: {filename}")
        QMessageBox.information(self, "Sukses", f"Video disimpan di {filename}")
      except Exception as e:
        self.node.get_logger().error(f"Gagal menyimpan video: {str(e)}")
        QMessageBox.critical(self, "Error", f"Gagal menyimpan video:\n{str(e)}")

  def generate_video(self, folder):
    png_files = sorted(glob.glob(os.path.join(folder, "*.png")))
    if not png_files:
      self.node.get_logger().warning("Tidak ditemukan file .png di folder tersebut.")
      return None

    # Buat file video sementara
    temp_video = tempfile.NamedTemporaryFile(delete=False, suffix='.mp4')
    temp_video_path = temp_video.name
    temp_video.close()

    # Definisi writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 10  # FPS dinaikkan ke 10
    video_writer = cv2.VideoWriter(temp_video_path, fourcc, fps, (800, 800))

    for file in png_files:
      img = cv2.imread(file)
      if img is None:
        self.node.get_logger().warning(f"Gagal membaca gambar: {file}")
        continue
      img_resized = cv2.resize(img, (800, 800))
      video_writer.write(img_resized)

    video_writer.release()
    self.node.get_logger().info(f"Video berhasil digenerate: {temp_video_path}")
    return temp_video_path

  def on_media_status_changed(self, status):
    # Jika video sudah selesai, reset teks tombol ke "Start video"
    if status == QMediaPlayer.EndOfMedia:
      self.btn_start_stop.setText("Start video")
      self.btn_pause_resume.setText("Pause video")
      
def main(args=None):
  rclpy.init(args=args)
  node = VideoGeneratorNode()

  app = QApplication(sys.argv)
  gui = VideoGeneratorGUI(node)
  gui.show()

  # Jika diperlukan, gunakan QTimer untuk memproses rclpy.spin_once secara periodik
  timer = QTimer()
  timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.001))
  timer.start(10)

  exit_code = app.exec()
  node.destroy_node()
  rclpy.shutdown()
  sys.exit(exit_code)


if __name__ == '__main__':
  main()