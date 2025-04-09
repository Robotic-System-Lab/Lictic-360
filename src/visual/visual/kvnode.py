import random
import colorsys
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from kivy.app import App
from kivy.clock import Clock
from kivy.uix.scatter import Scatter
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle
from kivy.graphics.texture import Texture
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.textinput import TextInput
from kivy.uix.label import Label
from kivy.uix.scrollview import ScrollView

# Contoh COLOR_MAP untuk nilai 0 s/d 50, dengan keterangan warna.
COLOR_MAP = {}
for i in range(50):
  hue = i / 50.0
  r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
  COLOR_MAP[i] = {"color": [r, g, b, 1], "name": f"Warna {i}"}

# Widget render berbasis tekstur: data grid dikonversi ke texture dan ditampilkan dengan satu objek Rectangle.
class TextureGrid(Widget):
  def __init__(self, data, color_map, **kwargs):
    super().__init__(**kwargs)
    self.data = data
    self.color_map = color_map
    # Default grid 40x40 (jika belum ada update dari pesan /map)
    self.map_width = 40
    self.map_height = 40
    self.size = (800, 800)
    # Buat texture dengan ukuran grid dan filter 'nearest' agar tidak blur saat diskalakan
    self.texture = Texture.create(size=(self.map_width, self.map_height), colorfmt='rgba')
    self.texture.mag_filter = 'nearest'
    with self.canvas:
      self.rect = Rectangle(texture=self.texture, pos=self.pos, size=self.size)
    self.bind(pos=self.update_rect, size=self.update_rect)
    self.update_grid()

  def update_rect(self, *args):
    self.rect.pos = self.pos
    self.rect.size = self.size

  def update_grid(self, *args):
    # Buat buffer untuk pixel (ukuran: map_width * map_height * 4 byte per pixel)
    buf = bytearray(self.map_width * self.map_height * 4)
    for i in range(self.map_height):
      for j in range(self.map_width):
        index = i * self.map_width + j
        # Jika data tidak lengkap, gunakan nilai default 0
        value = self.data[index] if index < len(self.data) else 0
        entry = self.color_map.get(value, {"color": [1, 1, 1, 1]})
        r, g, b, a = entry["color"]
        r_byte = int(r * 255)
        g_byte = int(g * 255)
        b_byte = int(b * 255)
        a_byte = int(a * 255)
        offset = (i * self.map_width + j) * 4
        buf[offset    ] = r_byte
        buf[offset + 1] = g_byte
        buf[offset + 2] = b_byte
        buf[offset + 3] = a_byte
    # Perbarui texture dengan buffer yang telah dibuat
    self.texture.blit_buffer(bytes(buf), colorfmt='rgba', bufferfmt='ubyte')
    # Balikkan texture secara vertikal untuk penyesuaian koordinat
    self.texture.flip_vertical()

# Scatter untuk mendukung zoom dan drag pada grid
class DraggableGrid(Scatter):
  def __init__(self, grid_widget, **kwargs):
    super().__init__(**kwargs)
    self.do_rotation = False
    self.do_translation = True
    self.do_scale = True
    self.size = grid_widget.size
    self.add_widget(grid_widget)

  def on_touch_down(self, touch):
    if touch.is_mouse_scrolling:
      if touch.button == 'scrolldown':
        factor = 1.1  # zoom in
      elif touch.button == 'scrollup':
        factor = 0.9  # zoom out
      else:
        factor = 1.0
      self.scale *= factor
      return True
    return super().on_touch_down(touch)

# Widget untuk menampilkan kotak warna pada legenda
class ColorBox(Widget):
  def __init__(self, color, **kwargs):
    super().__init__(**kwargs)
    self.size_hint = (None, None)
    self.size = (30, 30)
    with self.canvas:
      Color(*color)
      self.rect = Rectangle(pos=self.pos, size=self.size)
    self.bind(pos=self.update_rect, size=self.update_rect)

  def update_rect(self, *args):
    self.rect.pos = self.pos
    self.rect.size = self.size

# Item legenda: menampilkan square beserta nama warnanya
class LegendItem(BoxLayout):
  def __init__(self, number, entry, **kwargs):
    super().__init__(**kwargs)
    self.orientation = 'horizontal'
    self.size_hint_y = None
    self.height = 40
    self.add_widget(ColorBox(entry["color"]))
    self.add_widget(Label(text=entry["name"], size_hint_x=1, halign='left', valign='middle'))
    self.children[0].bind(size=self.update_text)

  def update_text(self, *args):
    self.children[0].texture_update()

# Kelas utama yang menampilkan GUI dan menyediakan method untuk mengupdate grid dari pesan OccupancyGrid.
class MainWidget(BoxLayout):
  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    # Base background warna #596968 (RGB 89,105,104)
    with self.canvas.before:
      Color(89/255, 105/255, 104/255, 1)
      self.bg_rect = Rectangle(pos=self.pos, size=self.size)
    self.bind(pos=self.update_bg, size=self.update_bg)
    self.orientation = 'horizontal'
    self.size_hint = (None, None)
    self.size = (1000, 850)  # panel kiri: 800x850, sidebar: 200x850
    self.last_update = 0
    self.min_update_interval = 1

    # Panel kiri: grid dan footer
    left_panel = BoxLayout(orientation='vertical', size_hint=(None, None), size=(800, 850))
    # Mulai dengan data dummy sebanyak 1600 elemen (default grid 40x40)
    dummy_data = [0] * (40 * 40)
    self.texture_grid = TextureGrid(dummy_data, COLOR_MAP)
    self.draggable = DraggableGrid(self.texture_grid)
    grid_container = Widget(size=(800, 800))
    grid_container.add_widget(self.draggable)
    left_panel.add_widget(grid_container)

    # Footer: tombol select path, form nama file, dan tombol save
    footer = BoxLayout(orientation='horizontal', size_hint=(None, None), size=(800, 50), padding=5, spacing=5)
    btn_select = Button(text='Select Path', size_hint=(None, 1), width=120)
    self.text_filename = TextInput(hint_text='File Name', multiline=False)
    btn_save = Button(text='Save', size_hint=(None, 1), width=80)
    btn_save.bind(on_release=self.save_grid)
    footer.add_widget(btn_select)
    footer.add_widget(self.text_filename)
    footer.add_widget(btn_save)
    left_panel.add_widget(footer)

    # Panel kanan: sidebar legenda scrollable dengan background putih
    sidebar_box = BoxLayout(orientation='vertical', size_hint_y=None, padding=10, spacing=5)
    sidebar_box.bind(minimum_height=sidebar_box.setter('height'))
    for i in range(50):
      entry = COLOR_MAP.get(i, {"color": [1, 1, 1, 1], "name": str(i)})
      legend_item = LegendItem(i, entry)
      legend_item.spacing = 10
      # Atur properti label agar teks hitam
      legend_item.children[0].color = (0, 0, 0, 1)
      legend_item.children[0].halign = 'left'
      legend_item.children[0].bind(size=lambda instance, value: setattr(instance, 'text_size', value))
      sidebar_box.add_widget(legend_item)
    sidebar_scroll = ScrollView(size_hint=(None, None), size=(200, 850))
    sidebar_scroll.add_widget(sidebar_box)
    with sidebar_scroll.canvas.before:
      Color(1, 1, 1, 1)
      self.sidebar_rect = Rectangle(pos=sidebar_scroll.pos, size=sidebar_scroll.size)
    sidebar_scroll.bind(pos=lambda instance, value: setattr(self.sidebar_rect, 'pos', value))
    sidebar_scroll.bind(size=lambda instance, value: setattr(self.sidebar_rect, 'size', value))

    self.add_widget(left_panel)
    self.add_widget(sidebar_scroll)

  def update_bg(self, *args):
    self.bg_rect.pos = self.pos
    self.bg_rect.size = self.size

  def save_grid(self, instance):
    file_name = self.text_filename.text.strip() or "grid_snapshot.png"
    target = None
    for child in self.walk():
      if isinstance(child, DraggableGrid):
        target = child
        break
    if target:
      target.export_to_png(file_name)
      print(f"Grid saved as {file_name}")
    else:
      print("Tidak menemukan grid untuk disimpan.")

  # Method yang dipanggil dari node ROS2 untuk mengupdate grid dari pesan OccupancyGrid
  def update_from_map(self, msg: OccupancyGrid):
    current_time = Clock.get_time()
    if current_time - self.last_update < self.min_update_interval:
      return
    self.last_update = current_time
    self.texture_grid.map_width = msg.info.width
    self.texture_grid.map_height = msg.info.height
    data = list(msg.data)
    expected = msg.info.width * msg.info.height
    if len(data) < expected:
      data += [0] * (expected - len(data))
    self.texture_grid.data = data
    self.texture_grid.update_grid()

# Node ROS2 yang mensubscribe topik /map dan menjadwalkan update GUI
class MapListener(Node):
  def __init__(self, update_callback):
    super().__init__('kvnode')
    self.subscription = self.create_subscription(
      OccupancyGrid,
      '/map',
      self.listener_callback,
      10)
    self.update_callback = update_callback

  def listener_callback(self, msg):
    # Jadwalkan update GUI pada thread utama Kivy
    Clock.schedule_once(lambda dt: self.update_callback(msg))

# Fungsi untuk menjalankan spin ROS2 di thread terpisah
def ros_spin(main_widget):
  rclpy.init(args=None)
  listener = MapListener(main_widget.update_from_map)
  rclpy.spin(listener)
  listener.destroy_node()
  rclpy.shutdown()

class SegnetApp(App):
  def build(self):
    main_widget = MainWidget()
    # Jalankan ROS2 spin di thread terpisah sehingga Kivy event loop tetap berjalan
    threading.Thread(target=ros_spin, args=(main_widget,), daemon=True).start()
    return main_widget

def main():
  SegnetApp().run()

if __name__ == '__main__':
  main()