import random
import colorsys

from kivy.app import App
from kivy.lang import Builder
from kivy.uix.scatter import Scatter
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.textinput import TextInput
from kivy.uix.label import Label

# Warna untuk angka 0 s/d 50
# Setiap nilai memiliki properti 'color' (RGBA) dan 'name'
COLOR_MAP = {}
for i in range(50):
  hue = i / 50.0  # Nilai hue antara 0 hingga hampir 1
  r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
  COLOR_MAP[i] = {"color": [r, g, b, 1], "name": f"Warna {i}"}

# Widget yang menggambar grid 40x40 berdasarkan data
class GridWidget(Widget):
  def __init__(self, data, color_map, **kwargs):
    super().__init__(**kwargs)
    self.data = data
    self.color_map = color_map
    # Ukuran grid tetap 800x800, setiap sel 20x20 (40x40)
    self.size = (800, 800)
    self.bind(pos=self.update_grid, size=self.update_grid)
    self.update_grid()

  def update_grid(self, *args):
    self.canvas.clear()
    with self.canvas:
      for i in range(40):
        for j in range(40):
          index = i * 40 + j
          value = self.data[index]
          entry = self.color_map.get(value, {"color": [1, 1, 1, 1]})
          Color(*entry["color"])
          # Menggambar kotak berukuran 20x20.
          # Untuk membuat grid dari atas ke bawah, y dihitung dari atas (800 - (i+1)*20).
          Rectangle(pos=(j * 20, 800 - (i + 1) * 20), size=(20, 20))

# Scatter kustom untuk mendukung zoom dan drag pada grid
class DraggableGrid(Scatter):
  def __init__(self, grid_widget, **kwargs):
    super().__init__(**kwargs)
    self.do_rotation = False
    self.do_translation = True
    self.do_scale = True
    # Menginisialisasi posisi dan ukuran dari grid
    self.size = grid_widget.size
    self.add_widget(grid_widget)

  def on_touch_down(self, touch):
    # Menangani scrollwheel untuk zoom dengan fokus pada kursor
    if touch.is_mouse_scrolling:
      if touch.button == 'scrolldown':
        factor = 1.1   # scrolldown -> zoom in
      elif touch.button == 'scrollup':
        factor = 0.9   # scrollup -> zoom out
      else:
        factor = 1.0
      # Mengubah skala dan menjaga posisi kursor (yang sederhana)
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
    # Kotak warna dan label nama
    self.add_widget(ColorBox(entry["color"]))
    self.add_widget(Label(text=entry["name"], size_hint_x=1, halign='left', valign='middle'))
    # Pastikan label melakukan refresh layout
    self.children[0].bind(size=self.update_text)
  
  def update_text(self, *args):
    self.children[0].texture_update()

# Layout utama aplikasi
class MainWidget(BoxLayout):
  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    # Set base background menjadi #596968 (RGB: 89,105,104)
    with self.canvas.before:
      Color(89/255, 105/255, 104/255, 1)
      self.bg_rect = Rectangle(pos=self.pos, size=self.size)
    self.bind(pos=self.update_bg, size=self.update_bg)
    self.orientation = 'horizontal'
    self.size_hint = (None, None)
    self.size = (1000, 850)  # grid (800x800 + footer 50) dan sidebar (200x850)

    # Panel kiri: grid dan footer dalam layout vertikal
    left_panel = BoxLayout(orientation='vertical', size_hint=(None, None), size=(800, 850))
    # Buat data random sebanyak 1600 (40x40) dengan nilai antara 0 s/d 17
    grid_data = [random.randint(0, 10) for _ in range(1600)]
    grid_widget = GridWidget(grid_data, COLOR_MAP)
    draggable = DraggableGrid(grid_widget)
    # Bungkus grid dalam layout dengan ukuran tetap
    grid_container = Widget(size=(800, 800))
    grid_container.add_widget(draggable)
    left_panel.add_widget(grid_container)

    # Footer: 800x50 dengan tiga elemen
    footer = BoxLayout(orientation='horizontal', size_hint=(None, None), size=(800, 50), padding=5, spacing=5)
    btn_select = Button(text='Select Path', size_hint=(None, 1), width=120)
    self.text_filename = TextInput(hint_text='File Name', multiline=False)
    btn_save = Button(text='Save', size_hint=(None, 1), width=80)
    btn_save.bind(on_release=self.save_grid)
    footer.add_widget(btn_select)
    footer.add_widget(self.text_filename)
    footer.add_widget(btn_save)
    left_panel.add_widget(footer)

    # Panel kanan: sidebar legenda (200x850) dengan background putih
    sidebar = BoxLayout(orientation='vertical', size_hint=(None, None), size=(200, 850), padding=10, spacing=5)
    with sidebar.canvas.before:
      Color(1, 1, 1, 1)
      self.sidebar_rect = Rectangle(pos=sidebar.pos, size=sidebar.size)
    sidebar.bind(pos=lambda instance, value: setattr(self.sidebar_rect, 'pos', value))
    sidebar.bind(size=lambda instance, value: setattr(self.sidebar_rect, 'size', value))
    
    from kivy.uix.scrollview import ScrollView
    sidebar_box = BoxLayout(orientation='vertical', size_hint_y=None, padding=10, spacing=5)
    sidebar_box.bind(minimum_height=sidebar_box.setter('height'))
    # Tambahkan item legenda untuk angka 0 sampai 49
    for i in range(50):
      entry = COLOR_MAP.get(i, {"color": [1, 1, 1, 1], "name": str(i)})
      legend_item = LegendItem(i, entry)
      legend_item.spacing = 10  # Add padding between the color square and the text
      label = legend_item.children[0]
      label.color = (0, 0, 0, 1)
      label.halign = 'left'
      # Ensure the label text aligns left and centered vertically by binding its size to text_size.
      label.bind(size=lambda instance, value: setattr(instance, 'text_size', value))
      sidebar_box.add_widget(legend_item)
    
    sidebar_scroll = ScrollView(size_hint=(None, None), size=(200, 850))
    sidebar_scroll.add_widget(sidebar_box)

    sidebar.add_widget(sidebar_scroll)
    self.add_widget(left_panel)
    self.add_widget(sidebar)

  def save_grid(self, instance):
    # Untuk menyimpan data grid sebagai gambar png.
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
      
  def update_bg(self, *args):
    self.bg_rect.pos = self.pos
    self.bg_rect.size = self.size

class SegnetApp(App):
  def build(self):
    from kivy.core.window import Window
    Window.size = (1000, 850)
    return MainWidget()

if __name__ == '__main__':
  SegnetApp().run()