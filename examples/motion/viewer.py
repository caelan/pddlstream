try:
    from Tkinter import Tk, Canvas, Toplevel, LAST
except ModuleNotFoundError:
    from tkinter import Tk, Canvas, Toplevel, LAST

import numpy as np


class PRMViewer(object):
    def __init__(self, width=500, height=500, title='PRM', background='tan'):
        tk = Tk()
        tk.withdraw()
        top = Toplevel(tk)
        top.wm_title(title)
        top.protocol('WM_DELETE_WINDOW', top.destroy)
        self.width = width
        self.height = height
        self.canvas = Canvas(top, width=self.width, height=self.height, background=background)
        self.canvas.pack()

    def pixel_from_point(self, point):
        (x, y) = point
        # return (int(x*self.width), int(self.height - y*self.height))
        return (x * self.width, self.height - y * self.height)

    def draw_point(self, point, radius=5):
        (x, y) = self.pixel_from_point(point)
        self.canvas.create_oval(x - radius, y - radius, x + radius, y + radius, fill='black')

    def draw_line(self, segment):
        (point1, point2) = segment
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_line(x1, y1, x2, y2, fill='black', width=2)

    def draw_arrow(self, point1, point2):
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_line(x1, y1, x2, y2, fill='black', width=2, arrow=LAST)

    def draw_rectangle(self, box, width=2, color='brown'):
        (point1, point2) = box
        (x1, y1) = self.pixel_from_point(point1)
        (x2, y2) = self.pixel_from_point(point2)
        self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, width=width)

    def draw_circle(self, center, radius, width=2, color='black'):
        (x1, y1) = self.pixel_from_point(np.array(center) - radius * np.ones(2))
        (x2, y2) = self.pixel_from_point(np.array(center) + radius * np.ones(2))
        self.canvas.create_oval(x1, y1, x2, y2, outline='black', fill=color, width=width)

    def clear(self):
        self.canvas.delete('all')


#################################################################

def get_delta(q1, q2):
    return np.array(q2) - np.array(q1)

def get_distance(q1, q2):
    return np.linalg.norm(get_delta(q1, q2))

def contains(q, box):
    (lower, upper) = box
    return np.greater_equal(q, lower).all() and np.greater_equal(upper, q).all()
    #return np.all(q >= lower) and np.all(upper >= q)

def sample_line(segment, step_size=.02):
    (q1, q2) = segment
    diff = get_delta(q1, q2)
    dist = np.linalg.norm(diff)
    for l in np.arange(0., dist, step_size):
        yield tuple(np.array(q1) + l * diff / dist)
    yield q2


def line_collides(line, box):  # TODO - could also compute this exactly
    return any(contains(p, box) for p in sample_line(line))


def is_collision_free(line, boxes):
    return not any(line_collides(line, box) for box in boxes)


def create_box(center, extents):
    (x, y) = center
    (w, h) = extents
    lower = (x - w / 2., y - h / 2.)
    upper = (x + w / 2., y + h / 2.)
    return np.array(lower), np.array(upper)


def sample_box(box):
    (lower, upper) = box
    return np.random.random(2) * (upper - lower) + lower

def inf_sequence():
    return iter(int, 1)

def draw_environment(obstacles, regions):
    viewer = PRMViewer()
    for box in obstacles:
        viewer.draw_rectangle(box, color='brown')
    for name, region in regions.items():
        if name != 'env':
            viewer.draw_rectangle(region, color='green')
    return viewer

def draw_solution(segments, obstacles, regions):
    viewer = draw_environment(obstacles, regions)
    if segments is None:
        return
    for line in segments:
        viewer.draw_line(line)
        #for p in [p1, p2]:
        for p in sample_line(line):
            viewer.draw_point(p, radius=2)


def draw_roadmap(roadmap, obstacles, regions):
    viewer = draw_environment(obstacles, regions)
    for line in roadmap:
        viewer.draw_line(line)
