import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'ro')

def init():
    ax.set_xlim(0, 2*np.pi)
    ax.set_ylim(-1, 1)
    return ln,

def update(frame):
    xdata.append(frame)
    ydata.append(np.sin(frame))
    ln.set_data(xdata, ydata)
    return ln,

ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
                    init_func=init, blit=True, interval=1)
plt.show()

# plots = plt.figure()
# plott0 = plt.subplot2grid((2,3),(0,0))
# pltt0, = plott0.plot([],[])
# #ln, = plt.plot([], [], 'ro')
# N = 100
# xdata = np.zeros(N)
# t = np.linspace(0, 2*np.pi, N)
# x = np.sin(t)
# count = 0
# t0, t1 = 0, 0

# def init():
#     plott0.set_xlim(0, 2*np.pi)
#     plott0.set_ylim(-1, 1)
#     return pltt0,

# def update(frame):
#     global count, t0, t1
#     global xdata

#     t1 = time.time()
#     print(t1 - t0)
#     t0 = time.time()

#     if (count >= N):
#         xdata = np.zeros(N)
#         count = 0

#     xdata = np.append(xdata, x[count])
#     xdata = xdata[1:]

#     pltt0.set_data(t, xdata)
#     count += 1
#     return pltt0,

# ani = FuncAnimation(plots, update, init_func=init, blit=True, interval=1)
# plt.show()