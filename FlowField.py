import numpy as np
from math import *
import matplotlib.pyplot as plt
import scipy.io
from mpl_toolkits.mplot3d import Axes3D
from sklearn.preprocessing import MinMaxScaler

#np.random.seed(88192019)
#

area = [-10, 30]
# minimum distance between the robot and an obstacle
rho_e = 1 # m
# radius of the robot
rho = 1 # m
rhoF = 0.2

def plot_flow_field(ax, area, goal, Fx,Fy):
    # creating a meshgrid for the flow field
    x, y = np.arange(area[0], area[1]), np.arange(area[0], area[1])
    xv, yv = np.meshgrid(x, y, sparse=False, indexing='ij')
    nx, ny = len(x), len(y)
    scaler = MinMaxScaler()
    scaler.fit(Fx)
    U=scaler.transform(Fx)
    V=scaler.transform(Fy)
    #  showing vector field direction
    Q = ax.quiver(xv, yv, U, V, units='width')
    ax.quiverkey(Q,  X=1, Y=1, U=1, label=r'$2 \frac{m}{s}$', labelpos='E',
                       coordinates='figure')

    # showing obstacles and goal locations
    x, y = goal
    circle1 = plt.Circle((x, y), 2, color='g')
    ax.add_artist(circle1)



class FlowField(object):
    def __init__(self):
        ##extracting ROMS flow field data
        mat = scipy.io.loadmat('data/UUVV711b.mat')
        lonmesh = mat['lonmesh']
        latmesh = mat['latmesh']
        Z = np.zeros((lonmesh.shape))
        self.m, self.n = lonmesh.shape  ## m and n represents the size of lonmesh and latmesh matrices
        L = mat['Depth']
        UU = mat['UU']
        VV = mat['VV']
        self.W = np.full(lonmesh.shape, 0)
        self.goal= [-4.96416653, -4.46200365]
        for k in range(0, 1):  # K represents the depth
            self.Z = np.full(lonmesh.shape, L[k])
            self.U = np.zeros((lonmesh.shape))
            self.V = np.zeros((latmesh.shape))
            for i in range(0, self.m):
                for j in range(0,self.n):
                    self.U[i][j] = UU[1][k][i][j];  ## index 1 represents the time, k represents the depth, i and j represent the longitude and latitude
                    self.V[i][j] = VV[1][k][i][j];

        #fig = plt.figure()
        #ax = fig.gca()
        #ax.quiver(lonmesh, latmesh, self.U, self.V, units='width')

    def __call__(self, r):
        Fx= self.U[r[0]][r[1]]
        Fy= self.V[r[0]][r[1]]
        return Fx, Fy


    def control_input(self, r):
        x, y = self.goal[0] - r[0], self.goal[1] - r[1]
        Fx, Fy = self(r)
        phi = atan2(Fy, Fx)
        u = tanh(x**2 + y**2)
        return [u, phi]

if __name__ == '__main__':
    FF = FlowField()
    fig, ax = plt.subplots()
    ax = plt.gca()
    ax.cla()  # clear things for fresh plot
    # ax1.set_title('Arrows scale with plot width, not view')
    plot_flow_field(ax,  area, FF.goal, FF.U, FF.V)

    plt.axis('equal')
    plt.show()

