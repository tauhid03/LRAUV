from ekf import plot_covariance_ellipse
import numpy as np
from queue import Queue
import os
from matplotlib import cbook, patches
import matplotlib.transforms as mtransforms
from ForceField import plot_vector_field, ForceField
class SimView(object):
    q = Queue()
    def __init__(self, plt, FF = None):
        self.plt = plt
        self.fig, self.ax = plt.subplots()
        self.xEst = np.zeros((4, 1))
        self.PEst = np.eye(4)
        self.hxEst = self.xEst
        self.u = np.zeros((2, 1))
        img_path = os.path.join(os.getcwd(),'icon/model3.png')
        with cbook.get_sample_data(img_path) as image_file:
            self.image = self.plt.imread(image_file)

        self.FF = FF
    def plot_vehicle(self, x, y , theta):
        offset_theta = np.deg2rad(90)
        theta = np.rad2deg(theta + offset_theta)%360
        offset_x, offset_y = 0, 0
        im = self.ax.imshow(self.image)
        transform = mtransforms.Affine2D().scale(0.75/30, 0.35/25).rotate_deg(theta).translate(x + offset_x, y + offset_y)
        trans_data = transform + self.ax.transData
        im.set_transform(trans_data)
    def update(self):
        # clear all screen
        self.plt.cla()
        plot_covariance_ellipse(self.xEst, self.PEst)
        self.plt.plot(self.hxEst[0, :].flatten(),
                 self.hxEst[1, :].flatten(), "-r")
        v, yawrate = self.u

        # plot dummy vehicle image
        x, y, theta= self.xEst[0, 0], self.xEst[1, 0], self.xEst[2, 0]
        self.plot_vehicle(x, y, theta)

        # plot vector field
        if (isinstance(self.FF, ForceField)):
            plot_vector_field(self.ax, [-10, 30], self.FF)
        # plot information
        self.plt.title("V = {}, Yaw = {}".format(v, yawrate))
        self.plt.axis([-10, 30, -10, 30])
        self.plt.grid(True)
        self.plt.pause(0.01)

    def __call__(self, args):
        self.xEst, self.PEst, self.u = args
        self.hxEst = np.hstack((self.hxEst, self.xEst))

if __name__ == '__main__':
    pass
