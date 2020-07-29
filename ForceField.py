import numpy as np
from math import *
import matplotlib.pyplot as plt

'''
This is the implementation of 
Motion Planning and Collision Avoidance using Navigation Vector Fields
by Dimitra Panagou
published in ICRA 2014
http://publish.illinois.edu/dpanagou/files/2014/07/Panagou_ICRA_14.pdf
'''
# np.random.seed(12345)
num_obstacles = 3
obstacle_radius = np.random.randint(2, 4, size=num_obstacles)
area = [-10, 30]
# minimum distance between the robot and an obstacle
rho_e = 1 # m
# radius of the robot
rho = 1 # m
rhoF = 0.2
rho_z = [ rho_oi + rho_e + rho for rho_oi in obstacle_radius]

def sampling_locations(num_samples, sample_dist = 15):
    samples = []
    for i in range(100):
        obs = np.random.uniform(area[0], area[1], size=(1,2))
        if(len(samples)<1):
            samples.append(obs)
        else:
            dist = min(map(lambda x: np.linalg.norm(obs - x), samples))
            if(dist>sample_dist):
                samples.append(obs)
                # print(dist, 'added')
            if(len(samples)>=num_samples):
                break
    return np.reshape(samples, (num_samples, 2))

def attractive_force_field(r, p):
    assert isinstance(r, list)
    assert isinstance(p, list)
    x, y = p[0] - r[0], p[1] - r[1]
    phi_i = atan2(-y, -x) + pi
    x, y = cos(phi_i), sin(phi_i)

    l = 2
    A = np.zeros((2,2))
    A[0, 1] = (l -1)*(x**2) -(y**2)
    A[0, 0] = l*x*y
    A[1, 1] = l*x*y
    A[1, 0] = (l - 1) * (y**2) - (x**2)
    p = np.array(p)
    F = np.matmul(A, p)
    return F
def replasive_force_field(r, obstacles):
    assert isinstance(r, list)
    A = np.zeros((2, 2))
    x, y = r
    for obs in obstacles:
        xoi, yoi = obs
        phi_i = atan2(-yoi, -xoi) + pi
        p = np.array([cos(phi_i), sin(phi_i)])
        A[0, 1] = -(y - yoi) ** 2
        A[0, 0] = (x - xoi) * (y - yoi)
        A[1, 1] = (x - xoi) * (y - yoi)
        A[1, 0] = - (x - xoi) ** 2
        Foi = np.matmul(A, p)
        yield Foi
def get_sigma(r, obstacles):
    def get_coefficients(beta_iz, beta_iF):
        a = 2
        b = -3*(beta_iz+beta_iF)
        c = 6*beta_iz*beta_iF
        d = beta_iz**2 *(beta_iz - 3*beta_iF)
        numerators = [a, b, c, d]
        denom = (beta_iz - beta_iF) ** 3
        result = list(map(lambda x: x/denom, numerators))
        return result
    assert isinstance(r, list)
    x, y = r

    for i, obs in enumerate(obstacles):
        xoi, yoi = obs
        rho_oi = obstacle_radius[i]
        beta_i = rho_oi**2 - (x-xoi)**2 - (y-yoi)**2
        # beta_iz = -2*rho_oi * (rho + rho_e) - (rho + rho_e)**2
        beta_iz = -2* rho_oi * (rho_z[i] - rho_oi) - (rho_z[i] - rho_oi)**2
        beta_iF = -2 * rho_oi * (rho_z[i] + rhoF - rho_oi) - (rho_z[i] + rhoF - rho_oi) ** 2
        if(beta_i<= beta_iF):
            yield 1
        elif ((beta_iF< beta_i) and (beta_i<beta_iz)):
            coeff = get_coefficients(beta_iz, beta_iF)
            a, b, c, d = coeff
            yield a*(beta_i**3) + b*(beta_i**2) + c*beta_i + d
        elif (beta_iz<= beta_i):
            yield 0

def vector_field(goal, r, obstacles):
    assert isinstance(goal, list)
    SIGMA = get_sigma(r, obstacles)
    repalsion = replasive_force_field(r, obstacles)
    FG, FOI = [], []
    for i, (sig, foi) in enumerate(zip(SIGMA, repalsion)):
        Fg = sig * attractive_force_field(r, goal)
        Foi =  foi * (1- sig)
        FG.append(Fg.T)
        FOI.append(Foi.T)
    # print(np.shape(FG), np.shape(FOI))
    F = np.prod(FG, axis=0) + np.sum(FOI, axis=0)
    return F
def control_input(g, r):
    x, y = g[0] - r[0], g[1] - r[1]
    rr = [x, y]
    F = vector_field(g, rr)
    Fx, Fy = F

    phi = atan2(Fy, Fx)
    u = tanh(x**2 + y**2)
    print(u, np.rad2deg(phi))
if __name__ == '__main__':
    samples = sampling_locations(num_samples=num_obstacles+1, sample_dist = 15)
    obstacles, goal = samples[:num_obstacles], samples[num_obstacles]
    print('obstacles')
    print(samples)
    print('goal')
    print(goal)
    x, y = np.arange(area[0], area[1]), np.arange(area[0], area[1])
    xv, yv = np.meshgrid(x, y, sparse=False, indexing='ij')
    nx, ny = len(x), len(y)
    U, V = np.zeros(xv.shape), np.zeros(xv.shape)
    for i in range(nx):
        for j in range(ny):
            r = [xv[i, j], yv[i, j]]
            F = vector_field(goal.tolist(), r, obstacles)
            U[i, j] = F[0]
            V[i, j] = F[1]

    fig, ax = plt.subplots()
    ax = plt.gca()
    ax.cla()  # clear things for fresh plot
    # ax1.set_title('Arrows scale with plot width, not view')
    Q = ax.quiver(xv, yv, U, V, units='width')
    qk = ax.quiverkey(Q,  X=1, Y=1, U=1, label=r'$2 \frac{m}{s}$', labelpos='E',
                       coordinates='figure')

    x, y = goal
    circle1 = plt.Circle((x, y), 2, color='g')
    ax.add_artist(circle1)
    for i , obs in enumerate(obstacles):
        x, y  = obs
        r = obstacle_radius[i]
        circle1 = plt.Circle((x, y), r, color='r')
        ax.add_artist(circle1)
    plt.axis('equal')
    plt.show()