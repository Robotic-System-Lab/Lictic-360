import random

from kivy.app import App
from kivy.lang import Builder
from kivy.properties import ListProperty
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.scatter import Scatter
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle
from kivy.uix.label import Label
from kivy.clock import Clock

# Fungsi mapping nilai ke warna (RGBA)
def get_color(val):
  color_map = {
    0: (1, 0, 0, 1),       # Merah
    1: (0, 1, 0, 1),       # Hijau
    2: (0, 0, 1, 1),       # Biru
    3: (1, 1, 0, 1),       # Kuning
    4: (1, 0.5, 0, 1),     # Orange
    5: (0.5, 0, 0.5, 1),   # Ungu
    6: (0, 1, 1, 1),       # Cyan
    7: (1, 0, 1, 1),       # Magenta
    8: (0.5, 1, 0, 1),     # Lime
    9: (1, 0.75, 0.8, 1),  # Pink
    10: (0, 0.5, 0.5, 1),   # Teal
    11: (0.9, 0.9, 0.98, 1),# Lavender
    12: (0.65, 0.16, 0.16, 1), # Brown
    13: (0.96, 0.96, 0.86, 1), # Beige
    14: (0.5, 0, 0, 1),     # Maroon
    15: (0.74, 0.99, 0.74, 1), # Mint
    16: (0.5, 0.5, 0, 1),    # Olive
    17: (1, 0.8, 0.6, 1),    # Apricot
    18: (0, 0, 0.5, 1),      # Navy
    19: (0.5, 0.5, 0.5, 1),  # Grey
    20: (1, 0.84, 0, 1)      # Gold
  }
  if val == 101:
    return (1, 1, 1, 1)   # Putih
  elif val == 102:
    return (0, 0, 0, 1)   # Hitam
  return color_map.get(val, (0, 0, 0, 1))

# --- Widget untuk grid 40x40 ---
class GridWidget(Widget):
  grid_data = ListProperty([])

  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    self.bind(pos=self.update_canvas,
              size=self.update_canvas,
              grid_data=self.update_canvas)

  def update_canvas(self, *args):
    self.canvas.clear()
    if not self.grid_data or len(self.grid_data) != 1600:
      return
    cell_w = self.width / 40.0
    cell_h = self.height / 40.0
    with self.canvas:
      for index, value in enumerate(self.grid_data):
        col = index % 40
        row = 39 - (index // 40)
        Color(*get_color(value))
        Rectangle(pos=(self.x + col * cell_w, self.y + row * cell_h),
                  size=(cell_w, cell_h))

# --- Scatter khusus untuk scroll dan zoom ---
class ScatterZoom(Scatter):
  # Non-rotasi, hanya translasi dan skala
  def on_touch_down(self, touch):
    if touch.is_mouse_scrolling:
      if touch.button == 'scrollup':
        self.scale *= 0.9
      elif touch.button == 'scrolldown':
        self.scale *= 1.1
      # Fitur zoom ke koordinat kursor dapat dikembangkan
      return True
    return super().on_touch_down(touch)

# --- Widget untuk Box Legenda warna ---
class ColorSquare(Widget):
	def __init__(self, color, size=(30,30), **kwargs):
		super().__init__(**kwargs)
		self.size_hint = (None, None)
		self.size = size
		with self.canvas:
			Color(*color)
			self.rect = Rectangle(pos=self.pos, size=self.size)
		self.bind(pos=self.update_rect, size=self.update_rect)

	def update_rect(self, *args):
		self.rect.pos = self.pos
		self.rect.size = self.size

# --- Widget untuk legenda warna ---
class LegendWidget(BoxLayout):
	def __init__(self, **kwargs):
		super().__init__(**kwargs)
		self.orientation = 'vertical'
		self.build_legend()

	def build_legend(self):
		# Tampilkan legenda untuk 0 s.d 20 dan khusus 101, 102
		for key in list(range(0, 21)) + [101, 102]:
			entry = BoxLayout(orientation='horizontal',
												size_hint_y=None,
												height=30,
												padding=(5,0))
			# Kotak warna dengan widget khusus
			square = ColorSquare(get_color(key))
			entry.add_widget(square)
			# Label keterangan. Ganti warna text bila perlu agar kontras.
			entry.add_widget(Label(text=str(key), size_hint_x=1, color=(0,0,0,1)))
			self.add_widget(entry)

# --- Layout Utama ---
class MainLayout(BoxLayout):
  grid_data = ListProperty([])

  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    self.init_grid_data()
    # Jadwalkan penyesuaian awal zoom setelah tampilan sudah terpanggil
    Clock.schedule_once(self.adjust_initial_zoom, 0)

  def init_grid_data(self):
    # Buat array 1600 dengan angka acak antara 0 s/d 20
    self.grid_data = [random.randint(0, 20) for _ in range(1600)]
    self.grid_data[0] = 101
    self.grid_data[1599] = 102
    # Beri nilai ke widget grid
    self.ids.grid_widget.grid_data = self.grid_data

  def adjust_initial_zoom(self, *args):
    # Dapatkan widget scatter dan widget grid
    scatter = self.ids.scatter
    grid = self.ids.grid_widget
    # Dapatkan ukuran kontainer scatter (sesuai ukuran BoxLayout yang mengandungnya)
    container_width, container_height = scatter.parent.size
    # Hitung faktor skala untuk memastikan grid (dengan ukuran aslinya) sepenuhnya terlihat
    scale_x = container_width / grid.width if grid.width else 1
    scale_y = container_height / grid.height if grid.height else 1
    optimal_scale = min(scale_x, scale_y)
    scatter.scale = optimal_scale
    print(f"Initial zoom set to {optimal_scale}")

  def open_filechooser(self):
      from kivy.uix.popup import Popup
      from kivy.uix.boxlayout import BoxLayout
      from kivy.uix.filechooser import FileChooserListView
      from kivy.uix.button import Button
      import os

      content = BoxLayout(orientation='vertical')
      default_path = os.getcwd()
      filechooser = FileChooserListView(path=default_path)
      content.add_widget(filechooser)

      btn_layout = BoxLayout(size_hint_y=None, height=40)
      select_btn = Button(text="Pilih", size_hint_x=1)
      cancel_btn = Button(text="Batal", size_hint_x=1)
      btn_layout.add_widget(select_btn)
      btn_layout.add_widget(cancel_btn)
      content.add_widget(btn_layout)

      popup = Popup(title="Pilih File/Path", content=content, size_hint=(0.9, 0.9))

      def select_file(instance):
          if filechooser.selection:
              selected = filechooser.selection[0]
              self.ids.filename_input.text = selected
          popup.dismiss()

      def cancel(instance):
          popup.dismiss()

      select_btn.bind(on_release=select_file)
      cancel_btn.bind(on_release=cancel)
      popup.open()

  def save_grid(self):
    filename = self.ids.filename_input.text
    if not filename:
      print("Masukkan nama file!")
      return
    # Simpan grid ke file png
    self.ids.grid_widget.export_to_png(filename)
    print(f"Grid disimpan dengan nama {filename}")

# --- Kivy Language ---
Builder.load_string('''
<MainLayout>:
  orientation: 'vertical'
  size: 800, 1000
  spacing: 5
  # Bagian atas: area grid 800x800
  BoxLayout:
    size_hint_y: None
    height: 800
    # Kami bungkus grid dalam ScatterZoom untuk mendukung zoom & scroll
    ScatterZoom:
      id: scatter
      do_rotation: False
      do_translation: True
      do_scale: True
      size_hint: None, None
      size: 800, 800
      GridWidget:
        id: grid_widget
        size: 800, 800
  # Bagian bawah: panel kontrol (save dan legenda)
  BoxLayout:
    size_hint_y: None
    height: 50
    spacing: 10
    padding: 10
    # Panel tombol dan form save
    BoxLayout:
      orientation: 'horizontal'
      spacing: 10
      Button:
        text: "Pilih Path"
        on_release: root.open_filechooser()
      TextInput:
        id: filename_input
        hint_text: "Nama File"
      Button:
        text: "Save"
        on_release: root.save_grid()
    # Panel legenda warna (disusun secara vertikal)
    LegendWidget:
      id: legend
      size_hint_x: None
      width: 200
''')

# --- App Utama ---
class KivyMapApp(App):
  def build(self):
    return MainLayout()

if __name__ == '__main__':
  KivyMapApp().run()