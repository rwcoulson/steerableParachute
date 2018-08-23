from math import *
import numpy as np
import serial
import FaBo9Axis_MPU9250 as MPU9250
from micropyGPS import MicropyGPS
from time import sleep
import wiringpi as wpi

r = 6.371e6

hemisphere = {'N': 1, 'S': -1, 'E':1, 'W':-1}

wpi.wiringPiSetupGpio()
wpi.pinMode(12,1)


def rotMatrix(axis, angle):
    'returns a rotation matrix about the given axis for the given angle'
    if axis == 'x' or axis == 'pitch':
        return np.array([ [1,0,0],
                          [0,cos(angle),-sin(angle)],
                          [0,sin(angle),cos(angle)]])
    if axis == 'y' or axis == 'roll':
        return np.array([ [cos(angle), 0,sin(angle)],
                          [0,          1,0         ],
                          [-sin(angle),0,cos(angle)]])
    if axis == 'z' or axis == 'yaw':
        return np.array([ [cos(angle),-sin(angle),0],
                          [sin(angle),cos(angle), 0],
                          [0,         0          ,1]])

def tiltCorrect(rawMag, rawG):
    ###not working
    '''outputs normalized 2D orientation in horizontal plane [E, N]
    inputs are 3D [x, y, z] vectors'''
    mag = rawMag / np.linalg.norm(rawMag)
    g = rawG / np.linalg.norm(rawG)
    roll = -asin(g[0][0])
    pitch = -atan2(g[1][0],g[2][0])
    rMat = rotMatrix('y',-roll)
    pMat = rotMatrix('x',-pitch)
    h = rMat @ pMat @ mag
    return h

def csvReadMat(filename):
    'extracts a numpy matrix stored in a csv file'
    mat = []
    f = open(filename,'r')
    data = f.readlines()
    f.close()
    for i in range(len(data)):
        data[i] = data[i].strip().split(',')
        for j in range(len(data[i])):
            data[i][j] = eval(data[i][j])
    return np.array(data)
            
def csvWrite(array, filename):
    'writes a list or array to a csv file'
    f = open(filename, 'w')
    for i in range(len(array)):
        if type(array[i]) == list or type(array[i]) == np.ndarray:
            for j in range(len(array[i])-1):
                f.write('{},'.format(array[i][j]))
            f.write('{}\n'.format(array[i][-1]))
        else:
            f.write('{}\n'.format(array[i]))
    f.close()

def gps2m(target, coord):
    '''converts gps coordinates to an x-y-z coordinate system
        utilizing arc-length formulas'''
    lat = coord[0]
    dlat = target[0] - coord[0]
    dlong = target[1] - coord[1]
    y = pi * r * (dlat / 180)
    x = pi * r * cos(lat * pi / 180) * (dlong/180)
    return [x,y]

def initFiles():    #initialize the error file
    'initializes the error tracking file'
    f = open('error.csv','w')
    f.write("timestamp,seconds in flight,x,y,z,x',y',z'\n")
    f.close()

class state(object):
    'takes sensor data and maintains a state estimate'
    def __init__(self):
        #initialize state from saved file
        try:
            file = open('state.csv','r')
            s = file.read().strip(',')
            if len(s) == 6:
                for i in range(len(s)):
                    s[i] = eval(s[i])
            file.close()
        except:
            self.X = np.array([0,0,0,0,0,0]).reshape([6,1])
        self.P = np.ones([6,6]) * 1000
        self.Y = np.array([0,0,0,0,0,0]).reshape([6,1])
        self.time = 0
        self.alt = 0
        self.dt = 1
        self.burstCount = 0
        self.burst = False

        self.windSpeed = []
        for i in range(1,311):
            #wind vector: [alt, vx, vy, sum_vx, sum_vy, N]
            self.windSpeed.append([i * 100, 0, 0, 0, 0, 0])
        
        #initialize IMU and GPS objects
        self.mpu = MPU9250.MPU9250()
        self.gps = MicropyGPS()

        #destination coordinates
        self.target = csvReadMat('target.csv').reshape([2])
        
        #set instrument variance
        self.R = csvReadMat('instVariance.csv')

    def Amatrix(self, theta = 0):
        'updates A matrix for time elapsed'
        self.A = np.eye(6)
        for i in range(3):
            self.A[i][i+3] = self.dt

    def Bmatrix(self, alt):
        'generates B matrix to correct for wind drift'
        i = alt // 100
        v = self.windSpeed[i][1:3]
        u = np.array([[0,0,0, v[0], v[1], 0]]).T
        B = np.zeros([6,6])
        for k in [3,4]:
            B[k-3][k] = self.dt
            B[k][k] = 1
        return B @ u
                
    def kFilter(self, Xprev, Pprev):
        'performs a Kalman filter and updates the state/covariance vectors'
        I = np.eye(6)
        Xp = (self.A @ Xprev).reshape([6,1]) + self.Bmatrix(self.X[2][0])
        Pp = self.A @ Pprev @ self.A.T
        K = np.zeros([6,6])
        for i in range(len(Pp)):
            for j in range(len(Pp[0])):
                if self.A[i][j] == 0:
                    Pp[i][j] = 0
                else:
                    K[i][j] = Pp[i][j] / (Pp[i][j] + self.R[i][j])
        X = Xp + K @ (self.Y - Xp)
        P = (I - K) @  Pp
        self.X = X
        self.P = P

    def logError(self):
        'logs the error into a file'
        err = (self.X - self.Y).T
        f = open('error.csv', 'a')
        f.write('{}:{}:{},'.format(self.gps.timestamp[0], self.gps.timestamp[1], self.gps.timestamp[2]))
        f.write('{},'.format(self.time))
        for i in range(5):
            f.write('{},'.format(err[0][i]))
        f.write('{}\n'.format(err[0][5]))
        f.close()

    def logWind(self):
        'logs the components of wind speed at 100m increment of altitude, averaged'
        #[alt, vx, vy, sum_vx, sum_vy, N]
        k = 0
        if self.X[2][0] >=0:
            k = self.X[2][0] // 100

        N = self.windSpeed[k][5]
        vxp = self.windSpeed[k][1] * N
        vyp = self.windSpeed[k][2] * N
        N += 1
        
        self.windSpeed[k][1] = (self.X[3][0] + vxp) / N
        self.windSpeed[k][2] = (self.X[4][0] + vyp) / N
        self.windSpeed[k][5] = N
        self.windSpeed[k][3] = 0
        self.windSpeed[k][4] = 0
        #adds up all wind components up to here; used for navigation algorithm
        for i in range(k):
            self.windSpeed[k][3] += self.windSpeed[i][3]
            self.windSpeed[k][4] += self.windSpeed[i][4]
        
    def measure(self):
        'takes measurements and populates Y(measurement) vector'
        #read IMU
        m = self.mpu.readMagnet()
        a = self.mpu.readAccel()
        mag = np.array([[m['x'], m['y'], m['z']]]).T
        accel = np.array([[a['x'], a['y'], a['z']]]).T

        self.dir = tiltCorrect(mag,accel)
        
        #read GPS
        data = [0,0]
        ser = serial.Serial('/dev/ttyS0', 4800, timeout = 1)
        for i in [0,1]:
            data[i] = ser.readline().decode('ascii').strip()
            for char in data[i]:
                self.gps.update(char)
        ser.close()
       
        #extract lat, long into degrees
        lat = (self.gps.latitude[0] + (self.gps.latitude[1] / 60)) * hemisphere[self.gps.latitude[2]]
        long = ( self.gps.longitude[0] + (self.gps.longitude[1] / 60)) * hemisphere[self.gps.longitude[2]]
        alt = self.gps.altitude

        #convert to xyz
        xy = gps2m(self.target, [lat,long])
        x = xy[0]
        y = xy[1]

        #find time elapsed
        t = self.gps.timestamp[0] * 3600 + self.gps.timestamp[1] * 60 + self.gps.timestamp[2]
        dt = t - self.time
        if dt != 0:
            self.dt = dt
        self.time = t

        #gps.speed is (knots, mph, km/h)
        v = self.gps.speed[2] * (5/18) #convert to m/s
        theta = -self.gps.course * pi/180 #convert to radians, correct for RHR
        vx = v * cos(theta)
        vy = v * sin(theta)
        try:
                vz = (alt - self.X[2][0]) / self.dt
        except:
                vz = self.X[5][0]
        self.Y = np.array([x, y, alt, vx, vy, vz]).reshape([6,1])

    def ascentLoop(self):
        'loop that runs during ascent'
        self.measure()
        self.logWind()
        
        if self.X[5][0] < 0 and self.X[2][0] > 500:
            self.burstCount += 1
            if self.burstCount >= 10:
                self.burst = True
                self.burstCount = 0
        else:
            self.burstCount = 0

        sleep(1.1)
            

    def descentLoop(self):
        'loop that runs during descent'
        self.measure()
        self.Amatrix()
        self.kFilter(self.X, self.P)
               
        if self.X[5][0] > 0:
            self.burstCount -= 1
            if self.burstCount <= -10:
                self.burst = False
                self.burstCount = 0
                csvWrite(self.windSpeed, 'windSpeed.csv')
        else:
            self.burstCount = 0

        sleep(1.1)

    def acquireGPS(self):
        'waits until GPS lock is acquired'
        lock = False
        while lock == False:
            self.measure()
            if self.gps.latitude[0:2] == (0, 0.0):
                wpi.digitalWrite(12,1)
                sleep(1)
                wpi.digitalWrite(12,0)
                sleep(1)
            else:
                lock = True
                wpi.digitalWrite(12,1)


##TO DO:
        #   instrument variance
        #   fix tilt corrections
        
