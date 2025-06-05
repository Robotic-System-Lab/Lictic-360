import sys
from rclpy.node import Node
import rclpy
import random
from PySide6.QtWidgets import (
  QApplication, QMainWindow, QGraphicsView, QGraphicsScene, QWidget,
  QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit, QFileDialog, QLabel,
  QFrame, QScrollArea
)
from PySide6.QtGui import QWheelEvent, QColor, QBrush, QPainter, QPixmap
from PySide6.QtCore import Qt, QRectF, QThread, Signal
from nav_msgs.msg import OccupancyGrid
import os
import datetime

# Custom GraphicsView dengan zoom dan drag (menggunakan AnchorUnderMouse)
class GridView(QGraphicsView):
  def __init__(self, scene, parent=None):
    super().__init__(scene, parent)
    self.setRenderHint(QPainter.Antialiasing)
    self.setDragMode(QGraphicsView.ScrollHandDrag)
    self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
  
  def wheelEvent(self, event: QWheelEvent):
    if event.angleDelta().y() < 0:
      factor = 1.25
    else:
      factor = 0.8
    self.scale(factor, factor)

# ROS2 Node yang berjalan di QThread terpisah
class ROS2ListenerNode(Node):
  def __init__(self, signal):
    super().__init__('qtnode')
    self.signal = signal
    self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

  def map_callback(self, msg):
    self.signal.emit(msg)

class ROS2Listener(QThread):
  map_received = Signal(object)
  def run(self):
    rclpy.init(args=[])
    node = ROS2ListenerNode(self.map_received)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class MainWindow(QMainWindow):
  def __init__(self):
    super().__init__()
    self.setWindowTitle("ROS2 Grid GUI dengan PySide6")
    self.save_path = ""
    container = QWidget()
    main_layout = QHBoxLayout(container)
    
    # Container kiri berisi grid dan footer
    left_container = QWidget()
    left_layout = QVBoxLayout(left_container)
    left_layout.setContentsMargins(0, 0, 0, 0)
    left_layout.setSpacing(0)
    
    self.start_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    self.save_indexer = 0
    self.scene = QGraphicsScene()
    self.grid_width = 40
    self.grid_height = 40
    self.cell_size = 800 / self.grid_width
    self.generate_default_grid()
    self.view = GridView(self.scene)
    self.view.setFixedSize(800, 800)
    
    left_layout.addWidget(self.view)
    
    # Sidebar legenda
    sidebar = QWidget()
    sidebar.setFixedSize(200, 850)
    sidebar_layout = QVBoxLayout(sidebar)
    sidebar_layout.setContentsMargins(5, 5, 5, 5)
    sidebar_layout.setSpacing(5)
    
    legend_area = QScrollArea()
    legend_area.setWidgetResizable(True)
    legend_widget = QWidget()
    legend_layout = QVBoxLayout(legend_widget)
    legend_layout.setContentsMargins(0, 0, 0, 0)
    legend_layout.setSpacing(2)
    
    for value in range(10):
      item = self.create_legend_item(value)
      legend_layout.addWidget(item)
    legend_area.setWidget(legend_widget)
    sidebar_layout.addWidget(legend_area)
    
    main_layout.addWidget(left_container)
    main_layout.addWidget(sidebar)
    
    self.setCentralWidget(container)
    
    self.ros2_thread = ROS2Listener()
    self.ros2_thread.map_received.connect(self.update_grid_from_map)
    self.ros2_thread.start()
  
  def generate_default_grid(self):
    self.grid_data = [random.randint(0, 10) for _ in range(self.grid_width * self.grid_height)]
    self.redraw_grid()
  
  # filepath: /home/lamp/workspaces/segnet/src/visual/visual/qtnode.py
  def redraw_grid(self):
    print(f"Redrawing grid (800x800)")
    # pixmap = QPixmap(800, 800)
    # pixmap.fill(Qt.white)
    # painter = QPainter(pixmap)
    # for row in range(self.grid_height):
    #     real_row = self.grid_height - 1 - row
    #     for col in range(self.grid_width):
    #         index = real_row * self.grid_width + col
    #         value = self.grid_data[index]
    #         if value == 99:
    #             color = QColor(50, 50, 50)
    #         elif value == -1:
    #             color = QColor('darkgray')
    #         elif value == 0:
    #             color = QColor('lightgray')
    #         else:
    #             ratio = (value - 1) / 9
    #             r = int(ratio * 255)
    #             b = 255 - r
    #             color = QColor(r, 100, b)
    #         rect = QRectF(col * self.cell_size, row * self.cell_size,
    #                       self.cell_size, self.cell_size)
    #         painter.fillRect(rect, QBrush(color))
    # painter.end()

    # Hanya update scene jika visualisasi diaktifkan
    # if getattr(self, "visual_enabled", True):
    #     self.scene.clear()
    #     self.scene.addPixmap(pixmap)

    self.save_indexer += 1
    if self.save_indexer % 3 == 0:
      print(f"Saving grid {self.save_indexer}")
      self.save_indexer = 0
      # image = QPixmap(800, 800)
      # image.fill(Qt.white)
      # painter = QPainter(image)
      # self.scene.render(painter)
      # painter.end()
      pixmap = QPixmap(800, 800)
      pixmap.fill(Qt.white)
      painter = QPainter(pixmap)
      for row in range(self.grid_height):
          real_row = self.grid_height - 1 - row
          for col in range(self.grid_width):
              index = real_row * self.grid_width + col
              value = self.grid_data[index]
              if value == 99:
                  color = QColor(50, 50, 50)
              elif value == -1:
                  color = QColor('darkgray')
              elif value == 0:
                  color = QColor('lightgray')
              else:
                  ratio = (value - 1) / 9
                  r = int(ratio * 255)
                  b = 255 - r
                  color = QColor(r, 100, b)
              rect = QRectF(col * self.cell_size, row * self.cell_size,
                            self.cell_size, self.cell_size)
              painter.fillRect(rect, QBrush(color))
      painter.end()
      
      folder = os.path.join(os.path.expanduser("~"), "lictic", "captured_map", str(self.start_time))
      os.makedirs(folder, exist_ok=True)
      timestamp_now = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
      full_path = f"{folder}/{timestamp_now}.png"
      pixmap.save(full_path, "PNG")
  
  def update_grid_from_map(self, msg):
    self.grid_width = msg.info.width
    self.grid_height = msg.info.height
    if self.grid_width > 0:
      self.cell_size = 800 / self.grid_width
    self.grid_data = list(msg.data)
    self.redraw_grid()
  
  def create_legend_item(self, value):
    item = QWidget()
    layout = QHBoxLayout(item)
    layout.setContentsMargins(5, 0, 5, 0)
    ratio = (value - 1) / 9
    r = int(ratio * 255)
    b = 255 - r
    color = QColor(r, 100, b)
    color_box = QFrame()
    color_box.setFixedSize(20, 20)
    color_box.setStyleSheet(f"background-color: {color.name()}; border: 1px solid black;")
    label = QLabel(str(value+1))
    layout.addWidget(color_box)
    layout.addWidget(label)
    return item

def main(args=None):
  app = QApplication(sys.argv)
  window = MainWindow()
  window.show()
  sys.exit(app.exec())

if __name__ == "__main__":
  main()