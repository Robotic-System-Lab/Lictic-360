from rclpy.node import Node
import rclpy
import datetime
import threading
import numpy as np
import tkinter as tk
from tkinter import filedialog
from PIL import Image
from nav_msgs.msg import OccupancyGrid

class TkinterMapNode(Node):
  def __init__(self):
    super().__init__('tknode')
    # Inisialisasi grid array dan dimensinya default (20x20)
    self.grid_array = [-1] * 400
    self.grid_width = 40
    self.grid_height = 40

    # Mapping angka ke warna sesuai permintaan
    self.colors = {
      0: 'white',
      1: 'green',
      2: 'blue',
      101: 'red',
      100: 'black',
      10: 'white',
      11: 'green',
      12: 'blue',
      13: 'red',
      14: 'black'  
    }

    # Inisialisasi variabel path penyimpanan
    self.save_path = ""
    # Variabel untuk zoom; base cell size
    self.base_cell_size = 20
    self.zoom = 1.0

    # Subscribe ke topik /map
    self.create_subscription(
      OccupancyGrid,
      '/map',
      self.map_callback,
      10
    )

    # Memulai GUI di thread terpisah agar tidak menghambat rclpy.spin()
    gui_thread = threading.Thread(target=self.init_gui)
    gui_thread.daemon = True
    gui_thread.start()

  def init_gui(self):
    self.root = tk.Tk()
    self.root.title("Map Grid")
    # Jendela fix 1000x1100 pixel (1000x1000 untuk grid, 100 untuk GUI lain)
    self.root.geometry("800x900")
    
    # Buat frame untuk canvas grid dengan ukuran fix 800x800
    grid_frame = tk.Frame(self.root, width=800, height=800)
    grid_frame.pack_propagate(False)
    grid_frame.pack()

    # Buat canvas dengan ukuran tampilan fix 800x800
    self.canvas = tk.Canvas(grid_frame, width=800, height=800)
    self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # Tambahkan scrollbar vertikal dan horizontal untuk canvas
    # vbar = tk.Scrollbar(self.root, orient=tk.VERTICAL, command=self.canvas.yview)
    # vbar.pack(side=tk.RIGHT, fill=tk.Y)
    # hbar = tk.Scrollbar(self.root, orient=tk.HORIZONTAL, command=self.canvas.xview)
    # hbar.pack(side=tk.BOTTOM, fill=tk.X)
    # self.canvas.config(yscrollcommand=vbar.set, xscrollcommand=hbar.set)
    
    # Bind event untuk zoom (untuk Linux: Button-4 dan Button-5; Windows/Mac: MouseWheel)
    self.canvas.bind("<Button-4>", self.zoom_handler)
    self.canvas.bind("<Button-5>", self.zoom_handler)
    self.canvas.bind("<MouseWheel>", self.zoom_handler)
    # Bind event untuk drag pada canvas
    self.canvas.bind("<ButtonPress-1>", self.start_drag)
    self.canvas.bind("<B1-Motion>", self.do_drag)

    # Tambahkan kontrol lain pada area bawah (tinggi 100 pixel)
    control_frame = tk.Frame(self.root, height=100)
    control_frame.pack(fill=tk.X, side=tk.BOTTOM)
    
    path_button = tk.Button(control_frame, text="Pilih Path", command=self.choose_path)
    path_button.pack(side=tk.LEFT, padx=10, pady=10)
    
    save_button = tk.Button(control_frame, text="Save", command=self.save_image)
    save_button.pack(side=tk.LEFT, padx=10, pady=10)
    
    # Gambar grid awal
    self.draw_grid(self.base_cell_size * self.zoom)
    self.root.mainloop()

  def start_drag(self, event):
    # Tandai awal drag
    self.canvas.scan_mark(event.x, event.y)

  def do_drag(self, event):
    # Geser canvas sesuai gerakan mouse
    self.canvas.scan_dragto(event.x, event.y, gain=1)

  def zoom_handler(self, event):
    old_zoom = self.zoom
    # Naikkan zoom jika scroll up, turunkan jika scroll down
    if self.zoom < 0.9 and (getattr(event, 'num', None) == 4 or event.delta > 0):
      self.zoom *= 1.1
    elif getattr(event, 'num', None) == 5 or event.delta < 0:
      self.zoom /= 1.1
    factor = self.zoom / old_zoom
    # Scale objek dengan tag "grid" relatif ke posisi kursor
    self.canvas.scale("grid", event.x, event.y, factor, factor)
    canvas_region_width = self.grid_width * self.base_cell_size * self.zoom
    canvas_region_height = self.grid_height * self.base_cell_size * self.zoom
    self.canvas.config(scrollregion=(0, 0, canvas_region_width, canvas_region_height))

  def choose_path(self):
    path = filedialog.askdirectory()
    if path:
      self.save_path = path
      print(f"Path penyimpanan dipilih: {self.save_path}")

  def save_image(self):
    if not self.save_path:
      print("Path penyimpanan belum dipilih!")
      return
    ps_file = f"{self.save_path}/temp.ps"
    self.canvas.postscript(file=ps_file)
    im = Image.open(ps_file)
    
    # Tambahkan legend untuk self.colors
    legend_height = 40 * ((len(self.colors) + 4) // 5)
    grid_width, grid_height = im.size
    new_im = Image.new("RGB", (grid_width, grid_height + legend_height), "white")
    new_im.paste(im, (0, 0))
    
    from PIL import ImageDraw, ImageFont
    draw = ImageDraw.Draw(new_im)
    
    col_width = grid_width // 5
    try:
      font = ImageFont.truetype("arial.ttf", 12)
    except IOError:
      font = ImageFont.load_default()
    
    for idx, key in enumerate(sorted(self.colors.keys())):
      floorheight = np.floor(idx/5) * 20
      color = self.colors[key]
      x_start = idx%5 * col_width
      rect_margin = 10
      top_margin = 14 if idx >= 5 else 0
      
      rect_x1 = x_start + rect_margin
      rect_y1 = grid_height + rect_margin + floorheight
      rect_x2 = x_start + col_width - rect_margin
      rect_y2 = grid_height + 25 + floorheight
      draw.rectangle([rect_x1, rect_y1+top_margin, rect_x2, rect_y2+top_margin], fill=color)
      
      text = f"{key}: {color}"
      text_x = x_start + 12
      text_y = grid_height + 27 + floorheight
      draw.text((text_x, text_y+top_margin), text, fill="black", font=font)
    
    # png_file = f"{self.save_path}/grid.png"
    timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    png_file = f"{self.save_path}/grid_{timestamp}.png"
    
    new_im.save(png_file, 'png')
    print(f"Grid dan legend berhasil disimpan sebagai {png_file}")

  def map_callback(self, msg):
    self.grid_array = list(msg.data)
    self.grid_width = msg.info.width
    self.grid_height = msg.info.height
    effective_cell = self.base_cell_size * self.zoom
    self.root.after(0, lambda: self.update_canvas(effective_cell))

  def update_canvas(self, cell_size):
    canvas_region_width = self.grid_width * cell_size
    canvas_region_height = self.grid_height * cell_size
    self.canvas.config(scrollregion=(0, 0, canvas_region_width, canvas_region_height))
    self.canvas.delete("all")
    self.draw_grid(cell_size)

  def draw_grid(self, cell_size):
    # Gambar grid dengan garis agak transparan (disimulasi dengan warna "#333333")
    for i in range(self.grid_height):
      for j in range(self.grid_width):
        idx = i * self.grid_width + j
        value = self.grid_array[idx] if idx < len(self.grid_array) else -1
        color = self.colors.get(value, 'gray')
        x1 = j * cell_size
        y1 = i * cell_size
        x2 = x1 + cell_size
        y2 = y1 + cell_size
        self.canvas.create_rectangle(
          x1, y1, x2, y2,
          fill=color,
          width=1,
          tags="grid"  # Tandai objek grid untuk keperluan scaling
        )

def main(args=None):
  rclpy.init(args=args)
  node = TkinterMapNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()