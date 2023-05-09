import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
import time
import serial
import struct

N =  25*3 #size of array (data to plot).
fs = 25 #sample frequency.
t0 = 0
plotindex = 0
gx = 0
gy = 0
gz = 0
ax = 0
ay = 0
az = 0
theta = 0
phi = 0
psi = 0

t = np.linspace(0, N/fs, N)
f = np.linspace(0, fs, N*3) #add zeros into the signal to improve spectral resolution.

datat0 = np.zeros(len(t))
datat1 = np.zeros(len(t))
datat2 = np.zeros(len(t))
dataf0 = np.zeros(len(f))
dataf1 = np.zeros(len(f))
dataf2 = np.zeros(len(f))

plots = plt.figure()
plt.subplots_adjust(bottom=0.2)
plott0 = plt.subplot2grid((2,3),(0,0))
plott1 = plt.subplot2grid((2,3),(0,1))
plott2 = plt.subplot2grid((2,3),(0,2))
plotf0 = plt.subplot2grid((2,3),(1,0))
plotf1 = plt.subplot2grid((2,3),(1,1))
plotf2 = plt.subplot2grid((2,3),(1,2))
plott0.grid(True)
plott1.grid(True)
plott2.grid(True)
plotf0.grid(True)
plotf1.grid(True)
plotf2.grid(True)
plott0.set_xlim(0, N/fs)
plott1.set_xlim(0, N/fs)
plott2.set_xlim(0, N/fs)
plotf0.set_xlim(0, fs)
plotf1.set_xlim(0, fs)
plotf2.set_xlim(0, fs)
plott0.set_xlabel('Time (s)')
plott1.set_xlabel('Time (s)')
plott2.set_xlabel('Time (s)')
plotf0.set_xlabel('Freq (Hz)')
plotf1.set_xlabel('Freq (Hz)')
plotf2.set_xlabel('Freq (Hz)')
pltt0, = plott0.plot([],[])
pltt1, = plott1.plot([],[])
pltt2, = plott2.plot([],[])
pltf0, = plotf0.plot([],[])
pltf1, = plotf1.plot([],[])
pltf2, = plotf2.plot([],[])

class ResetPlot():
    def gyro(self, event):
        global plots, plott0, plott1, plott2, plotf0, plotf1, plotf2
        global datat0, datat1, datat2
        global plotindex

        plotindex = 0

        datat0 = np.zeros(N)
        datat1 = np.zeros(N)
        datat2 = np.zeros(N)

        plots.suptitle('GYROSCOPE', fontsize=12)
        plott0.set_title('X axis')
        plott1.set_title('Y axis')
        plott2.set_title('Z axis')
        plott0.set_ylabel('deg/s')
        plott1.set_ylabel('deg/s')
        plott2.set_ylabel('deg/s')
        plotf0.set_ylabel('fftGX')
        plotf1.set_ylabel('fftGY')
        plotf2.set_ylabel('fftGZ')
        plott0.set_ylim(-200, 200)
        plott1.set_ylim(-200, 200)
        plott2.set_ylim(-200, 200)
        plotf0.set_ylim(-10, 4000)
        plotf1.set_ylim(-10, 4000)
        plotf2.set_ylim(-10, 4000)
    
    def accel(self, event):
        global plots, plott0, plott1, plott2, plotf0, plotf1
        global datat0, datat1, datat2
        global plotindex

        plotindex = 1

        datat0 = np.zeros(N)
        datat1 = np.zeros(N)
        datat2 = np.zeros(N)

        plots.suptitle('ACCELEROMETER', fontsize=12)
        plott0.set_title('X axis')
        plott1.set_title('Y axis')
        plott2.set_title('Z axis')
        plott0.set_ylabel('m/ss')
        plott1.set_ylabel('m/ss')
        plott2.set_ylabel('m/ss')
        plotf0.set_ylabel('fftAX')
        plotf1.set_ylabel('fftAY')
        plotf2.set_ylabel('fftAZ')
        plott0.set_ylim(-20, 20)
        plott1.set_ylim(-20, 20)
        plott2.set_ylim(-20, 20)
        plotf0.set_ylim(-10, 300)
        plotf1.set_ylim(-10, 300)
        plotf2.set_ylim(-10, 300)

    def euler(self, event):
        global plots, plott0, plott1, plott2, plotf0, plotf1, plotf2
        global datat0, datat1, datat2
        global plotindex

        plotindex = 2

        datat0 = np.zeros(N)
        datat1 = np.zeros(N)
        datat2 = np.zeros(N)

        plots.suptitle('EULER', fontsize=12)
        plott0.set_title('ROLL')
        plott1.set_title('PITCH')
        plott2.set_title('YAW')
        plott0.set_ylabel('deg')
        plott1.set_ylabel('deg')
        plott2.set_ylabel('deg')
        plotf0.set_ylabel('fftROLL')
        plotf1.set_ylabel('fftPITCH')
        plotf2.set_ylabel('fftYAW')
        plott0.set_ylim(-200, 200)
        plott1.set_ylim(-200, 200)
        plott2.set_ylim(-200, 200)
        plotf0.set_ylim(-10, 4000)
        plotf1.set_ylim(-10, 4000)
        plotf2.set_ylim(-10, 4000)

reset = ResetPlot()
gb = plt.axes([0.2, 0.05, 0.15, 0.075])
ab = plt.axes([0.425, 0.05, 0.2, 0.075])
eb = plt.axes([0.7, 0.05, 0.1, 0.075])
gyrob = Button(gb, 'GYROSCOPE')
accelb = Button(ab, 'ACCELEROMETER')
eulerb = Button(eb, 'EULER')
gyrob.on_clicked(reset.gyro)
accelb.on_clicked(reset.accel)
eulerb.on_clicked(reset.euler)

s = serial.Serial(port='COM3', baudrate=115200)

def geteuler(q):
    (x, y, z, w) = (q[0], q[1], q[2], q[3])
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)*180/np.pi
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    yaw = np.arcsin(t2)*180/np.pi
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    pitch = np.arctan2(t3, t4)*180/np.pi
    return [roll, pitch, yaw]

def getdata():
    global gx, gy, gz
    global ax, ay, az
    global theta, phi, psi
    global datat0, datat1, datat2, dataf0, dataf1, dataf2

    #t0 = time.time()
    s.write('IMU'.encode())
    data = struct.unpack('<ffffffffff', s.read(10*4))
    #t1 = time.time()
    #print(t1 - t0)

    gx = data[0]
    gy = data[1]
    gz = data[2]
    ax = data[3]
    ay = data[4]
    az = data[5]
    # q0 = data[6]
    # q1 = data[7]
    # q2 = data[8]
    # q3 = data[9]

    [theta, phi, psi] = geteuler(data[6:])

    # theta = -np.arcsin(2*q1*q3+2*q0*q2)*180/np.pi #roll
    # phi = np.arctan2(2*q2*q3-2*q0*q1, 2*q0**2+2*q3**2-1)*180/np.pi #pitch
    # psi = np.arctan2(2*q1*q2-2*q0*q3, 2*q0**2+2*q1**2-1)*180/np.pi #yaw

    if (plotindex == 0):
        datat0 = np.append(datat0, gx)
        datat1 = np.append(datat1, gy)
        datat2 = np.append(datat2, gz)
        datat0 = datat0[1:]
        datat1 = datat1[1:]
        datat2 = datat2[1:]
    elif (plotindex == 1):
        datat0 = np.append(datat0, ax)
        datat1 = np.append(datat1, ay)
        datat2 = np.append(datat2, az)
        datat0 = datat0[1:]
        datat1 = datat1[1:]
        datat2 = datat2[1:]
    elif (plotindex == 2):
        datat0 = np.append(datat0, theta)
        datat1 = np.append(datat1, phi)
        datat2 = np.append(datat2, psi)
        datat0 = datat0[1:]
        datat1 = datat1[1:]
        datat2 = datat2[1:]
    dataf0 = np.abs(np.fft.fft(datat0, len(f)))
    dataf1 = np.abs(np.fft.fft(datat1, len(f)))
    dataf2 = np.abs(np.fft.fft(datat2, len(f)))
    #time.sleep(0.001)

def update(frames):  
    global N, fs, t0
    
    #measure the time of getdata(1). about 0.008 sec.
    # t0 = time.time()
    # getdata()
    # t1 = time.time()
    # print(t1 - t0)

    getdata()

    if (plotindex == 0):
        pltt0.set_data(t, datat0)
        pltt1.set_data(t, datat1)
        pltt2.set_data(t, datat2)
        pltf0.set_data(f, dataf0)
        pltf1.set_data(f, dataf1)
        pltf2.set_data(f, dataf2)
    elif (plotindex == 1):
        pltt0.set_data(t, datat0)
        pltt1.set_data(t, datat1)
        pltt2.set_data(t, datat2)
        pltf0.set_data(f, dataf0)
        pltf1.set_data(f, dataf1)
        pltf2.set_data(f, dataf2)
    elif (plotindex == 2):
        pltt0.set_data(t, datat0)
        pltt1.set_data(t, datat1)
        pltt2.set_data(t, datat2)
        pltf0.set_data(f, dataf0)
        pltf1.set_data(f, dataf1)
        pltf2.set_data(f, dataf2)

    #measure the time per draw. about 0.05 sec.
    # t1 = time.time()
    # print(t1 - t0)
    # t0 = time.time()

    return pltt0, pltt1, plott2, pltf0, pltf1, pltf2,

ani = FuncAnimation(plots, update,interval=1, init_func=reset.gyro(0), blit=True, repeat_delay=True)
plt.show()

# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.widgets import Button
# from matplotlib.animation import FuncAnimation
# #from matplotlib.lines import Line2D
# import time
# import serial
# import struct

# fs = 100 #sample frequency.
# N = 100 #size of array (data to plot).
# gx = 0
# gy = 0
# gz = 0
# ax = 0
# ay = 0
# az = 0
# theta = 0
# phi = 0
# psi = 0

# gyrox = np.zeros(N)
# gyroy = np.zeros(N)
# gyroz = np.zeros(N)
# accelx = np.zeros(N)
# accely = np.zeros(N)
# accelz = np.zeros(N)
# roll = np.zeros(N)
# pitch = np.zeros(N)
# yaw = np.zeros(N)

# plots = plt.figure()
# plt.subplots_adjust(bottom=0.2)
# plott0 = plt.subplot2grid((2,3),(0,0))
# plott1 = plt.subplot2grid((2,3),(0,1))
# plott2 = plt.subplot2grid((2,3),(0,2))
# plotf0 = plt.subplot2grid((2,3),(1,0))
# plotf1 = plt.subplot2grid((2,3),(1,1))
# plotf2 = plt.subplot2grid((2,3),(1,2))

# pltt0, = plott0.plot([],[])
# pltt1, = plott1.plot([],[])
# pltt2, = plott2.plot([],[])
# pltf0, = plotf0.plot([],[])
# pltf1, = plotf1.plot([],[])
# pltf2, = plotf2.plot([],[])

# time = np.linspace(0, N/fs, N)
# freq = np.linspace(0, fs, N)

# class Reset():
#     def resetplots(self):
#         global plott0, plott1, plott2, plotf0, plotf1, plotf2
#         global pltt0, pltt1, pltt2, pltf0, pltf1, pltf2

#         plott0 = plt.subplot2grid((2,3),(0,0))
#         plott1 = plt.subplot2grid((2,3),(0,1))
#         plott2 = plt.subplot2grid((2,3),(0,2))
#         plotf0 = plt.subplot2grid((2,3),(1,0))
#         plotf1 = plt.subplot2grid((2,3),(1,1))
#         plotf2 = plt.subplot2grid((2,3),(1,2))
#         plott0.grid(True)
#         plott1.grid(True)
#         plott2.grid(True)
#         plotf0.grid(True)
#         plotf1.grid(True)
#         plotf2.grid(True)
#         plott0.set_xlim(0, N/fs)
#         plott1.set_xlim(0, N/fs)
#         plott2.set_xlim(0, N/fs)
#         plotf0.set_xlim(0, fs)
#         plotf1.set_xlim(0, fs)
#         plotf2.set_xlim(0, fs)
#         plott0.set_xlabel('Time (s)')
#         plott1.set_xlabel('Time (s)')
#         plott2.set_xlabel('Time (s)')
#         plotf0.set_xlabel('Freq (Hz)')
#         plotf1.set_xlabel('Freq (Hz)')
#         plotf2.set_xlabel('Freq (Hz)')

#         pltt0, = plott0.plot([],[])
#         pltt1, = plott1.plot([],[])
#         pltt2, = plott2.plot([],[])
#         pltf0, = plotf0.plot([],[])
#         pltf1, = plotf1.plot([],[])
#         pltf2, = plotf2.plot([],[])

#     def gyro(self, event):
#         global plots, plott0, plott1, plott2, plotf0, plotf1, plotf2
#         global gyrox, gyroy, gyroz

#         gyrox = np.zeros(N)
#         gyroy = np.zeros(N)
#         gyroz = np.zeros(N)
        
#         self.resetplots()
        
#         plots.suptitle('GYROSCOPE', fontsize=12)
#         plott0.set_ylim(-100, 100)
#         plott1.set_ylim(-100, 100)
#         plott2.set_ylim(-100, 100)
#         plotf0.set_ylim(-100, 100)
#         plotf1.set_ylim(-100, 100)
#         plotf2.set_ylim(-100, 100)
        
#     def accel(self, event):
#         global plots, plott0, plott1, plott2, plotf0, plotf1, plotf2
#         global accelx, accely, accelz

#         accelx = np.zeros(N)
#         accely = np.zeros(N)
#         accelz = np.zeros(N)

#         self.resetplots()

#         plots.suptitle('ACCELEROMETER', fontsize=12)
#         plott0.set_ylim(-16, 16)
#         plott1.set_ylim(-16, 16)
#         plott2.set_ylim(-16, 16)
#         plotf0.set_ylim(-16, 16)
#         plotf1.set_ylim(-16, 16)
#         plotf2.set_ylim(-16, 16)

#     def euler(self, event):
#         global plots, plott0, plott1, plott2, plotf0, plotf1, plotf2
#         # global accelx, accely, accelz

#         # accelx = np.zeros(N)
#         # accely = np.zeros(N)
#         # accelz = np.zeros(N)

#         self.resetplots()

#         plots.suptitle('EULER', fontsize=12)
#         plott0.set_ylim(-180, 180)
#         plott1.set_ylim(-180, 180)
#         plott2.set_ylim(-180, 180)
#         plotf0.set_ylim(-180, 180)
#         plotf1.set_ylim(-180, 180)
#         plotf2.set_ylim(-180, 180)

# reset = Reset()
# gb = plt.axes([0.2, 0.05, 0.15, 0.075])
# ab = plt.axes([0.425, 0.05, 0.2, 0.075])
# eb = plt.axes([0.7, 0.05, 0.1, 0.075])
# gyrob = Button(gb, 'GYROSCOPE')
# accelb = Button(ab, 'ACCELEROMETER')
# eulerb = Button(eb, 'EULER')
# gyrob.on_clicked(reset.gyro)
# accelb.on_clicked(reset.accel)
# eulerb.on_clicked(reset.euler)

# s = serial.Serial(port='COM3', baudrate=115200)

# def getdata():
#     global gx, gy, gz
#     global ax, ay, az
#     global theta, phi, psi

#     s.write('IMU'.encode())
#     data = struct.unpack('<ffffffffff', s.read(10*4))

#     gx = data[0]
#     gy = data[1]
#     gz = data[2]
#     ax = data[3]
#     ay = data[4]
#     az = data[5]
#     q0 = data[6]
#     q1 = data[7]
#     q2 = data[8]
#     q3 = data[9]

#     theta = -np.arcsin(2*q1*q3+2*q0*q2)*180/np.pi #roll
#     phi = np.arctan2(2*q2*q3-2*q0*q1, 2*q0**2+2*q3**2-1)*180/np.pi #pitch
#     psi = np.arctan2(2*q1*q2-2*q0*q3, 2*q0**2+2*q1**2-1)*180/np.pi #yaw

# def update(frames):  
#     global gyrox, gyroy, gyroz
#     global accelx, accely, accelz
#     global roll, pitch, yaw
#     global N, fs
   
#     getdata()

#     gyrox = np.append(gyrox, gx)
#     gyroy = np.append(gyroy, gy)
#     gyroz = np.append(gyroz, gz)
#     accelx = np.append(accelx, ax)
#     accely = np.append(accely, ay)
#     accelz = np.append(accelz, az)
#     roll = np.append(roll, theta)
#     pitch = np.append(pitch, phi)
#     yaw = np.append(yaw, phi)
#     gyrox = gyrox[1:]
#     gyroy = gyroy[1:]
#     gyroz = gyroz[1:]
#     accelx = accelx[1:]
#     accely = accely[1:]
#     accelz = accelz[1:]
#     roll = roll[1:]
#     pitch = pitch[1:]
#     yaw = yaw[1:]

#     pltt0.set_data(time, gyrox)
#     pltt1.set_data(time, gyroy)
#     pltt2.set_data(time, gyroz)
#     #pltf0.set_data(freq, gyroxf)
#     #pltf1.set_data(freq, gyroyf)
#     #pltf2.set_data(freq, gyrozf)

#     return pltt0, pltt1, plott2,

# # def initial (frames):
# #     plott0.set_ylim(-100, 100)
# #     plott1.set_ylim(-100, 100)
# #     plott2.set_ylim(-100, 100)
# #     plotf0.set_ylim(-100, 100)
# #     plotf1.set_ylim(-100, 100)
# #     plotf2.set_ylim(-100, 100)

# #     pltt0.set_data(time, gyrox)
# #     pltt1.set_data(time, gyroy)
# #     pltt2.set_data(time, gyroz)
# #     #pltf0.set_data(freq, gyroxf)
# #     #pltf1.set_data(freq, gyroyf)
# #     #pltf2.set_data(freq, gyrozf)

# #     return pltt0, pltt1, plott2

# ani = FuncAnimation(plots, update, interval=1, init_func=reset.gyro(0), blit=True, repeat=True)
# plt.show()
