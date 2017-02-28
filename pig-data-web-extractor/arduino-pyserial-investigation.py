import serial
from Messure import Messure
import time
import numpy as np
from matplotlib import pyplot as plt
import matplotlib
ser = serial.Serial('/dev/ttyACM0', 9600)
import matplotlib.gridspec as gridspec
messures = list()

start = time.time();

SIZE = 500

def backup(messures):
    pass


plt.ion()
gyr_x = [0] * SIZE
gyr_y = [0] * SIZE
gyr_z = [0] * SIZE

acc_x = [0] * SIZE
acc_y = [0] * SIZE
acc_z = [0] * SIZE

mean_gyr_x = 0
mean_gyr_y = 0
mean_gyr_z = 0


mag_x = [0] * SIZE
mag_y = [0] * SIZE
mag_z = [0] * SIZE



i = 0


def plot_drawings():
    print 'gear , ' + 'mean_x', 'mean_y', 'mean_z'
    print np.mean(gyr_x), np.mean(gyr_y), np.mean(gyr_z)
    #mean


    mean_gyr_x = [np.mean(gyr_x)] * SIZE
    mean_gyr_y = [np.mean(gyr_y)] * SIZE
    mean_gyr_z = [np.mean(gyr_z)] * SIZE


    fig = plt.figure(figsize=(3, 3))
    G = gridspec.GridSpec(3, 3)
    axes_gyr_1 = plt.subplot(G[0, 0])
    axes_gyr_1.plot(np.arange(len(gyr_x)), gyr_x, 'r-', np.arange(len(mean_gyr_x)), mean_gyr_x, 'b')
    axes_gyr_2 = plt.subplot(G[1, 0])
    axes_gyr_2.plot(np.arange(len(gyr_y)), gyr_y, 'g-', np.arange(len(mean_gyr_y)), mean_gyr_y, 'r')
    axes_gyr_3 = plt.subplot(G[2, 0])
    axes_gyr_3.plot(np.arange(len(gyr_z)), gyr_z, 'b-', np.arange(len(mean_gyr_z)), mean_gyr_z, 'r')

    axes_acc_1 = plt.subplot(G[0, 1])
    axes_acc_1.plot(np.arange(len(acc_x)), acc_x, 'r-')
    axes_acc_2 = plt.subplot(G[1, 1])
    axes_acc_2.plot(np.arange(len(acc_y)), acc_y, 'g-')
    axes_acc_3 = plt.subplot(G[2, 1])
    axes_acc_3.plot(np.arange(len(acc_z)), acc_z, 'b-')

    axes_mag_1 = plt.subplot(G[0, 2])
    axes_mag_1.plot(np.arange(len(mag_x)), mag_x, 'r-')
    axes_mag_2 = plt.subplot(G[1, 2])
    axes_mag_2.plot(np.arange(len(mag_y)), mag_y, 'g-')
    axes_mag_3 = plt.subplot(G[2, 2])
    axes_mag_3.plot(np.arange(len(mag_z)), mag_z, 'b-')

    plt.show(fig)  # update the plot




while True:
    try:

        # plt.pause(0.01)
        # plt.clf()
        ln = ser.readline()
        print ln
        mes = ln.rstrip().split(',')
        print mes
        messures.append(Messure(time.time(), mes))

        if(len(mes) == 9):
            i=i+1
            gyr_x.append(float(mes[6]))
            gyr_y.append(float(mes[7]))
            gyr_z.append(float(mes[8]))

            acc_x.append(float(mes[0]))
            acc_y.append(float(mes[1]))
            acc_z.append(float(mes[2]))

            mag_x.append(float(mes[3]))
            mag_y.append(float(mes[4]))
            mag_z.append(float(mes[5]))

            del gyr_x[0]
            del gyr_y[0]
            del gyr_z[0]
            del acc_x[0]
            del acc_y[0]
            del acc_z[0]
            del mag_x[0]
            del mag_y[0]
            del mag_z[0]

        if(i>=SIZE+10):

            break;

        stop = time.time()

        if(stop-start > 10000):
            backup(messures)
            messures = list()
    except IndexError as e:
        print e
        print 'index error'



plot_drawings()