
class Messure():
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

    def __init__(self, timestamp, list ):
        self.timestamp = timestamp;
        self.acc_x = list[0]
        self.acc_y = list[1]
        self.acc_z = list[2]

        self.mag_x = list[3]
        self.mag_y = list[4]
        self.mag_z = list[5]


        self.gyr_x = list[6]
        self.gyr_y = list[7]
        self.gyr_z = list[8]

    def __repr__(self):
        return self.timestamp+":"+self.gyr_x+":"+self.gyr_y+":"+self.gyr_z

