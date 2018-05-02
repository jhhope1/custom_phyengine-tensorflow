import environment
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ion()
endtime = 5

def plot_robot(points):
    num_points = np.shape(points)[0]
    x = np.zeros((num_points), dtype=np.float64)
    y = np.zeros((num_points), dtype=np.float64)
    z = np.zeros((num_points), dtype=np.float64)
    for i in range(num_points):
        x[i] = points[i].flatten()[0]
        y[i] = points[i].flatten()[1]
        z[i] = points[i].flatten()[2]
    ax.scatter(x, y, z)
    ax.set_zlim(0, 0.5)
    ax.set_xlim(points[0][0]-0.5, points[0][0]+0.5)
    ax.set_ylim(points[0][1]-0.5, points[0][1]+0.5)
    plt.draw()

#if __name__ == "__main__":
def test(actor):
    env = environment.Env()
    state_size = env.state_size
    action_size = env.action_size
    env.reset()
    #  env.R.body.wb = np.array([[1.], [-1.], [1.]])
    #  env.R.body.vs = np.array([[0.], [2.], [1.]])
    for t in range(int(endtime/env.R.dtime)):
        print('t = ', t)
        action = actor(np.reshape(env.state, [1, env.state_size]), batch_size=1)
        #  np.zeros((12), dtype=np.float64)
        next_state, reward, done = env.step(action)
        points = np.concatenate([np.reshape(env.R.body.Rnow, (4, 3)), np.reshape(env.R.joints, (12, 3))], axis=0)
        if (t==0):
            plt.show()
        if (done == 1):
            print(done)
            break
        plot_robot(points)
        plt.pause(0.01)
        ax.clear()