from __future__ import print_function

try:
    from Tkinter import Tk, Canvas, Toplevel
except ImportError:
    from tkinter import Tk, Canvas, Toplevel

# NOTE - this will overwrite (but remember) existing drawings
# TODO - try PyGame, PyCairo, or Pyglet

SUCTION_WIDTH = 1.5 # 1.01 | 1.5
STEM_WIDTH = 1.
STEM_HEIGHT = 2.5

PIXEL_BUFFER = 10
ENV_HEIGHT = 1.

def get_width(interval):
    return interval[1] - interval[0]

class ContinuousTMPViewer(object):
    def __init__(self, suction_height, regions, tl_x=0, tl_y=0, width=400, height=200, title='Grid', background='tan'):
        self.tk = Tk()
        # tk.geometry('%dx%d+%d+%d'%(width, height, 100, 0))
        self.tk.withdraw()
        self.top = Toplevel(self.tk)
        self.top.wm_title(title)
        self.top.protocol('WM_DELETE_WINDOW', self.top.destroy)

        self.suction_height = suction_height
        self.regions = regions
        self.width = width
        self.height = height
        self.canvas = Canvas(self.top, width=self.width, height=self.height, background=background)
        self.canvas.pack()
        # self.center()
        self.move_frame(tl_x, tl_y)

        max_width = max(map(get_width, regions.values())) # Really should take width of max minus min
        self.dist_to_pixel = (self.width - 2 * PIXEL_BUFFER) / max_width  # Maintains aspect ratio
        self.dist_width = self.width / self.dist_to_pixel
        self.dist_height = self.height / self.dist_to_pixel
        self.ground_height = self.height - self.dist_to_pixel * ENV_HEIGHT
        self.robot_dist = self.dist_height / 2.

        self.dynamic = []
        self.static = []
        self.draw_environment()

    def center(self):
        self.top.update_idletasks()
        w = self.top.winfo_screenwidth()
        h = self.top.winfo_screenheight()
        size = tuple(int(_) for _ in self.top.geometry().split('+')[0].split('x'))
        x = w / 2 - size[0] / 2
        y = h / 2 - size[1] / 2
        self.top.geometry("%dx%d+%d+%d" % (size + (x, y)))

    def move_frame(self, x, y):
        self.top.update_idletasks()
        size = tuple(int(_) for _ in self.top.geometry().split('+')[0].split('x'))
        self.top.geometry("%dx%d+%d+%d" % (size + (x, y)))

    def scale_x(self, x):  # x \in [-self.dist_width/2, self.dist_width/2]
        return self.width / 2. + self.dist_to_pixel * x

    def scale_y(self, y):  # y \in [0, self.dist_height]
        return self.ground_height - self.dist_to_pixel * y

    def draw_block(self, x, y, width, height, name='', color='blue'):
        self.dynamic.extend([
            self.canvas.create_rectangle(self.scale_x(x - width / 2.), self.scale_y(y),
                                         self.scale_x(x + width / 2.), self.scale_y(y + height),
                                         fill=color, outline='black', width=2),
            self.canvas.create_text(self.scale_x(x), self.scale_y(y + height / 2), text=name),
        ])

    # def draw_holding(self, x, width, height, color='blue'):
    #     self.holding = self.canvas.create_rectangle(self.scale_x(x - width / 2.),
    #                                                 self.scale_y(self.robot_dist - SUCTION_HEIGHT / 2 - height),
    #                                                 self.scale_x(x + width / 2.),
    #                                                 self.scale_y(self.robot_dist - SUCTION_HEIGHT / 2),
    #                                                 fill=color, outline='black', width=2)

    def draw_region(self, region, name='', color='red'):
        x1, x2 = map(self.scale_x, region)
        y1, y2 = self.ground_height, self.height
        self.static.extend([
            self.canvas.create_rectangle(x1, y1, x2, y2,
                                         fill=color, outline='black', width=2),
            self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text=name),
        ])

    def draw_environment(self):
        # TODO: automatically draw in order
        self.static = []
        for name, region in sorted(self.regions.items(), key=lambda pair: get_width(pair[1]), reverse=True):
            self.draw_region(region, name=name, color=name)


    def draw_robot(self, x, y, name='', color='yellow'):  # TODO - could also visualize as top grasps instead of side grasps
        #y = self.robot_dist
        self.dynamic.extend([
            self.canvas.create_rectangle(self.scale_x(x - SUCTION_WIDTH / 2.),
                                         self.scale_y(y - self.suction_height / 2.),
                                         self.scale_x(x + SUCTION_WIDTH / 2.),
                                         self.scale_y(y + self.suction_height / 2.),
                                         fill=color, outline='black', width=2),
            self.canvas.create_text(self.scale_x(x), self.scale_y(y), text=name),
            self.canvas.create_rectangle(self.scale_x(x - STEM_WIDTH / 2.),
                                         self.scale_y(y + self.suction_height / 2.),
                                         self.scale_x(x + STEM_WIDTH / 2.),
                                         self.scale_y(y + self.suction_height / 2. + STEM_HEIGHT),
                                         fill=color, outline='black', width=2),
        ])

    def clear_state(self):
        for part in self.dynamic:
            self.canvas.delete(part)
        self.dynamic = []

    def clear_all(self):
        self.canvas.delete('all')

    def save(self, filename):
        # TODO: screen recording based on location like I did before
        # TODO: only works on windows
        # self.canvas.postscript(file='%s.ps'%filename, colormode='color')
        #from PIL import ImageGrab
        try:
            import pyscreenshot as ImageGrab
        except ImportError:
            return None
        x, y = self.top.winfo_x(), 2*self.top.winfo_y()
        width, height = self.top.winfo_width(), self.top.winfo_height() # winfo_width, winfo_reqheight
        path = filename + '.png'
        print('Saved', path)
        ImageGrab.grab((x, y, x+width, y+height)).save(path)
        return path
        #os.system("screencapture -R {},{},{},{} {}".format(
        #    225, 200, 600, 500, image_path))
