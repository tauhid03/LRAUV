import numpy as np
from threading import Thread
from pynput.keyboard import Key, Listener
from env import TethysEnv
from SimView import SimView
import matplotlib.pyplot as plt
from time import sleep
from ForceField import sampling_locations, ForceField

class KeyboardInput(Thread):
    def __init__(self, env):
        Thread.__init__(self)
        self.env = env
    def on_press(self, key):
        try:
            if str(key) in self.env.values:
                self.env.q.put(str(key))
            else:
                self.env.q.put(key.char)
        except:
            pass
    def on_release(self, key):
        # print('{0} release'.format(key))
        if key == Key.esc:
            # Stop listener
            return False
    def run(self) -> None:
        global terminate
        # Collect events until released
        with Listener(
                on_press=self.on_press,
                on_release=self.on_release) as listener:
            listener.join()
        self.env.terminate = True

if __name__ == '__main__':
    print('LRAUV project')

    # showing vector field
    num_obstacles = 3
    samples = sampling_locations(num_samples=num_obstacles + 1, sample_dist=15)
    obstacles, goal = samples[:num_obstacles], samples[num_obstacles]
    FF = ForceField(goal.tolist(), obstacles)


    sim = SimView(plt, FF)
    tethys = TethysEnv(sim, FF)
    IOthread = KeyboardInput(tethys)
    IOthread.start()
    tethys.start()


    while not tethys.terminate:
        while not sim.q.empty():
            data = sim.q.get()
            sim(data)
        sim.update()
        sleep(0.01)
