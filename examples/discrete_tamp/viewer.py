try:
    from Tkinter import Tk, Canvas, Toplevel
except ImportError:
    from tkinter import Tk, Canvas, Toplevel

import colorsys


# NOTE - this will overwrite (but remember) existing drawings
# TODO - try PyGame, PyCairo, or Pyglet

MAX_ROWS = 5
MAX_COLS = 9

COLORS = ['red', 'orange', 'yellow', 'green', 'blue', 'violet', 'white', 'black']

#def spaced_colors(n, s=1, v=1):
#    return [colorsys.hsv_to_rgb(h, s, v) for h in np.linspace(0, 1, n, endpoint=False)]

def tk_from_rgb(rgb):
    assert all(0 <= c < 256 for c in rgb)
    return "#%02x%02x%02x" % rgb

#COLORS = list(map(tk_from_rgb, [(255, 0, 0)]))

class DiscreteTAMPViewer(object):
    def __init__(self, rows, cols, width=500, height=250, side=25,
                 block_buffer=10, title='Grid', background='tan', draw_fingers=False):
        assert (rows <= MAX_ROWS)
        assert (cols <= MAX_COLS)

        tk = Tk()
        tk.withdraw()
        top = Toplevel(tk)
        top.wm_title(title)
        top.protocol('WM_DELETE_WINDOW', top.destroy)

        self.width = width
        self.height = height
        self.rows = rows
        self.cols = cols
        self.canvas = Canvas(top, width=self.width, height=self.height, background=background)
        self.canvas.pack()
        self.side = side
        self.block_buffer = block_buffer
        self.draw_fingers = draw_fingers
        self.cells = {}
        self.robot = []
        self.draw_environment()

    def transform_r(self, r):
        return self.table_y1 + r * (self.side + 2 * self.block_buffer) + 2 * self.block_buffer + self.side / 2

    def transform_c(self, c):
        # assert r >= 0 and r < self.width
        return self.table_x1 + c * (self.side + 2 * self.block_buffer) + 2 * self.block_buffer + self.side / 2

    def draw_environment(self, table_color='lightgrey', bin_color='grey'):
        table_width = self.cols * (self.side + 2 * self.block_buffer) + 2 * self.block_buffer
        table_height = self.rows * (self.side + 2 * self.block_buffer) + 2 * self.block_buffer

        border_buffer = 50
        #self.table_x1 = border_buffer
        self.table_y1 = self.height - table_height - border_buffer
        self.table_x1 = self.width/2-table_width/2
        #self.table_y1 = self.height/2-table_height/2

        bin_width = 20
        self.environment = [
            self.canvas.create_rectangle(self.table_x1, self.table_y1,
                                         self.table_x1 + table_width, self.table_y1 + table_height,
                                         fill=table_color, outline='black', width=2),
            self.canvas.create_rectangle(self.table_x1 - bin_width, self.table_y1,
                                         self.table_x1, self.table_y1 + table_height,
                                         fill=bin_color, outline='black', width=2),
            self.canvas.create_rectangle(self.table_x1 + table_width, self.table_y1,
                                         self.table_x1 + table_width + bin_width, self.table_y1 + table_height,
                                         fill=bin_color, outline='black', width=2),
            self.canvas.create_rectangle(self.table_x1, self.table_y1 + table_height,
                                         self.table_x1 + table_width, self.table_y1 + table_height + bin_width,
                                         fill=bin_color, outline='black', width=2),
            self.canvas.create_rectangle(self.table_x1 - bin_width, self.table_y1 + table_height,
                                         self.table_x1 + table_width + bin_width,
                                         self.table_y1 + table_height + bin_width,
                                         fill=bin_color, outline='black', width=2),
        ]

        pose_radius = 2
        for r in range(self.rows):
            for c in range(self.cols):
                x = self.transform_c(c)
                y = self.transform_r(r)
                self.environment.append(self.canvas.create_oval(x - pose_radius, y - pose_radius,
                                                                x + pose_radius, y + pose_radius, fill='black'))

    def draw_robot(self, r, c, color='yellow'):
        # TODO - could also visualize as top grasps instead of side grasps
        grasp_buffer = 3 # 0 | 3 | 5
        finger_length = self.side + grasp_buffer  # + self.block_buffer
        finger_width = 10
        gripper_length = 20
        if self.draw_fingers:
            gripper_width = self.side + 2 * self.block_buffer + finger_width
        else:
            gripper_width = self.side
        stem_length = 50
        stem_width = 20

        x = self.transform_c(c)
        y = self.transform_r(r) - self.side / 2 - gripper_length / 2 - grasp_buffer
        finger_x = gripper_width / 2 - finger_width / 2
        self.robot = [
            self.canvas.create_rectangle(x - stem_width / 2., y - stem_length,
                                         x + stem_width / 2., y,
                                         fill=color, outline='black', width=2),
            self.canvas.create_rectangle(x - gripper_width / 2., y - gripper_length / 2.,
                                         x + gripper_width / 2., y + gripper_length / 2.,
                                         fill=color, outline='black', width=2),
        ]
        if self.draw_fingers:
            self.robot += [
                self.canvas.create_rectangle(x + finger_x - finger_width / 2., y,
                                             x + finger_x + finger_width / 2., y + finger_length,
                                             fill=color, outline='black', width=2),
                self.canvas.create_rectangle(x - finger_x - finger_width / 2., y,
                                             x - finger_x + finger_width / 2., y + finger_length,
                                             fill=color, outline='black', width=2),
            ]


    def draw_block(self, r, c, name='', color='red'):
        x = self.transform_c(c)
        y = self.transform_r(r)
        self.cells[(x, y)] = [
            self.canvas.create_rectangle(x - self.side / 2., y - self.side / 2.,
                                         x + self.side / 2., y + self.side / 2.,
                                         fill=color, outline='black', width=2),
            self.canvas.create_text(x, y, text=name),
        ]

    # def delete(self, (x, y)):
    #  if (x, y) in self.cells:
    #    self.canvas.delete(self.cells[(x, y)])

    def clear(self):
        self.canvas.delete('all')

    def save(self, filename):
        # self.canvas.postscript(file='%s.ps'%filename, colormode='color')
        from PIL import ImageGrab
        ImageGrab.grab((0, 0, self.width, self.height)).save(filename + '.jpg')
