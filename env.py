from queue import Queue
from time import sleep
import numpy as np
from threading import Thread
from ekf import observation, ekf_estimation
from time import sleep

# plt.gcf().canvas.mpl_connect('key_release_event',
#                              lambda event: [exit(0) if event.key == 'escape' else None])
V_MAX = 5
YAW_MAX = 2

class TethysEnv(Thread):
    values = ["Key.up", "Key.down", "Key.left", "Key.right", "w", "s"]
    terminate = False
    q = Queue()

    def __init__(self, view):
        self.view = view
        Thread.__init__(self)
        self.v = 0
        self.yawrate = 0
        self.layer = 0
        # EKF inputs
        self.xEst = np.zeros((4, 1))
        self.xTrue = np.zeros((4, 1))
        self.u = np.zeros((2, 1))
        self.PEst = np.eye(4)
        self.xDR = np.zeros((4, 1))  # Dead reckoning


    def update(self):
        self.xTrue, z, self.xDR, ud = observation(self.xTrue, self.xDR, self.u)
        self.xEst, self.PEst = ekf_estimation(self.xEst, self.PEst, z, ud)


    def fixInput(self, u, u_max):
        sign = u/abs(u) if abs(u)>0 else 0
        return sign * min(abs(u), u_max)
    def step(self, a_ind):
        # print(a_ind)
        v_incr = 0.1
        yaw_incr = 0.1
        if(a_ind == 0):
            self.v += v_incr
            self.yawrate = 0
        elif (a_ind == 1):
            self.v -= v_incr
            self.yawrate = 0
        elif (a_ind == 2):
            self.yawrate += yaw_incr
            self.v = 0
        elif (a_ind == 3):
            self.yawrate -= yaw_incr
            self.v = 0
        elif (a_ind == 4):
            self.layer += 1
        elif (a_ind == 5):
            self.layer -= 1

        self.v = self.fixInput(self.v, V_MAX)
        self.yawrate = self.fixInput(self.yawrate, YAW_MAX)
        self.u = np.array([[self.v], [self.yawrate]])





    def run(self) -> None:
        '''
            The key idea is to get keyboard inputs (a.k.a key) from different thread and
            call step function if the key matches to a desire string in the values.
        '''
        while (not self.terminate):
            while (not self.q.empty()):
                key = self.q.get()
                if key in self.values:
                    index = self.values.index(key)
                    # print(' {0} pressed {1}'.format(key, index))
                    self.step(a_ind=index)
                    self.update()
                    self.view.q.put([self.xEst, self.PEst, self.u])
                    sleep(0.01)
            sleep(0.01)




