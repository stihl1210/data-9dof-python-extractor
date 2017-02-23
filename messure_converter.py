import patoolib
import os
import numpy as np
import itertools as it

def unpack(path, name):
    if not os.path.exists(path):
        os.mkdir(path)
        patoolib.extract_archive(name, outdir=path)

def extract_data(pathname, name):
    read_data = None
    with open(pathname+name, 'r') as f:
        read_data = f.read()
    f.closed

    return read_data






def grouper(n, iterable, fillvalue=None):
    "grouper(3, 'ABCDEFG', 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return it.izip_longest(fillvalue=fillvalue, *args)


def get_messures(dataString):
    import re

    valuesTab = filter(None, re.split("[, \*\@]+", dataString))
    filtered = list()
    for val in valuesTab:
        if (val != '' and len(val)<=2):
            filtered.append(val)

    values = list()
    for i in range(len(filtered)):
        if (i % 2 == 1):
            fill = ( int(filtered[i - 1], 16) )
            fill2 = ( int(filtered[i] , 16))

            filled = fill | fill2 << 8

            # print filtered[i-1], filtered[i], filled

            if(filled>32767):
                values.append(filled-65536)
            else:
                values.append(filled)

    dimX = len(values)/9
    values = list(grouper(9, values))


    numpyValues = np.zeros( (dimX,9), dtype=np.float32)
    mes = messure_converter();

    for i in range(dimX):
        numpyValues[i] = mes.gen_messure(values[i])

    return numpyValues






class messure_converter():
    A_SCALE = 0b000
    M_SCALE = 0b00
    G_SCALE = 0b00

    def __init__(self ):
        self.timestamp = 0;
        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0

        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0


        self.gyr_x = 0
        self.gyr_y = 0
        self.gyr_z = 0

        self.id = 0
    def gen_messure(self, listOfValues):
        self.acc_x = ax = self.getAcceleration(listOfValues[0])
        self.acc_y = ay = self.getAcceleration(listOfValues[1])
        self.acc_z = az= self.getAcceleration(listOfValues[2])

        self.mag_x = mx =  self.getMagnitude(listOfValues[3])
        self.mag_y = my = self.getMagnitude(listOfValues[4])
        self.mag_z = mz = self.getMagnitude(listOfValues[5])

        self.gyr_x = gx = self.getGyroscope(listOfValues[6])
        self.gyr_y = gy = self.getGyroscope(listOfValues[7])
        self.gyr_z = gz = self.getGyroscope(listOfValues[8])

        return np.array((self.acc_x,
        self.acc_y,
        self.acc_z,

        self.mag_x,
        self.mag_y ,
        self.mag_z ,


        self.gyr_x ,
        self.gyr_y ,
        self.gyr_z ), dtype=np.float32)


    def getAcceleration(self, val):
            if(self.A_SCALE == 0b100):
                acc =16.0 / 32768.0
            else:
                acc = ((self.A_SCALE + 1.0) * 2.0) / 32768.0;

            return val * acc

    def getMagnitude(self,val):
            if(self.M_SCALE == 0):
                mag = 2.0 / 32768.0

            else:
                mag = ((self.M_SCALE << 2) / 32768.0)
            return mag * val

    def getGyroscope(self,val):
        if(self.G_SCALE == 0b00):
            gyro = 245.0 / 32768.0
        elif(self.G_SCALE == 0b01):
            gyro = 500.0 / 32768.0
        elif(self.G_SCALE == 0b10):
            gyro = 2000.0 / 32768.0
        else:
            return 0
        return gyro * val
    def __repr__(self):
        asd = "acc: "  + str(self.acc_x) + ", " + str(self.acc_y)+ ", " +  str(self.acc_z)
        asd += " mag: " + ", " + str(self.mag_x) +  ", " + str(self.mag_y) +  ", " + str(self.mag_z)
        asd +=  " gyr: " + ", " + str(self.gyr_x) +  ", " + str(self.gyr_y) +  ", " + str(self.gyr_z)
        return asd

def concatenateBytes( lo, hi):
    return hi << 8 |  lo

    return gyro * val



unpack('data-unpacked','capture.rar')


def save_stats_from_messures(dir, statsdir, param1):

    from os import listdir
    from os.path import isfile, join

    if not os.path.exists(statsdir):
        os.mkdir(statsdir)

    onlyfiles = [f for f in listdir(dir) if isfile(join(dir, f))  ]
    stats = list()

    file_comment = "mean for dataset : acc,mag,gyr - "
    for i in onlyfiles:
        file_comment+= i + ", "
        print ('Analysys for file: ' + i)
        messures = get_messures(extract_data('./data-unpacked/', i ))
        np.savetxt( statsdir+ i +'_unpacked', (messures), delimiter=',',fmt='%f')
        stats.append(np.mean(messures, axis=0))

    np.savetxt( (statsdir + param1), stats, delimiter='\t',fmt='%f', footer=file_comment)


save_stats_from_messures('./data-unpacked/', './data-stats/' , 'stats.txt')





