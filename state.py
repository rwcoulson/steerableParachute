from math import *
import numpy as np
import serial
import FaBo9Axis_MPU9250 as MPU9250
from micropyGPS import MicropyGPS

r = 6.371e6

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
    '''outputs normalized 2D orientation in horizontal plane [E, N]
    inputs are 3D [x, y, z] vectors'''
    procMag = [0, 0]
    mag = rawMag / np.linalg.norm(rawMag)
    g = rawG / np.linalg.norm(rawG)
    roll = -asin(g[0])
    pitch = -atan2(g[1],g[2])
    rMat = rotMatrix('y',-roll)
    pMat = rotMatrix('x',-pitch)
    h = rMat @ pMat @ mag
    return [h[0],h[1]]


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

        self.windSpeed = []
        for i in range(201):
            self.windSpeed.append([i * 100, 0, 0, 0])

        
        #initialize IMU and GPS objects
        self.mpu = MPU9250.MPU9250()
        self.gps = MicropyGPS()
##        #implement: destination coordinates in external file
        #implement: retrieve instrument variance from external file

    def Amatrix(self, dt = 1, theta = 0):
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
                
    def kFilter(self, Xprev, Pprev, A, Bu, R, Y):
        'performs a Kalman filter and updates the state/covariance vectors'
        I = np.eye(6)
        Xp = (A @ Xprev).reshape([6,1]) + Bu
        Pp = A @ Pprev @ A.T
        K = np.zeros([6,6])
        for i in range(len(Pp)):
            for j in range(len(Pp[0])):
                if A[i][j] == 0:
                    Pp[i][j] = 0
                else:
                    K[i][j] = Pp[i][j] / (Pp[i][j] + R[i][j])
        X = Xp + K @ (Y - Xp)
        P = (I - K) @  Pp
        self.X = X
        self.P = P

    def logError(self):
        err = (self.X - self.Y).T
        f = open('error.csv', 'a')
        f.write('timestamp,{}:{}:{},'.format(self.gps.timestamp[0], self.gps.timestamp[1], self.gps.timestamp[2]))
        f.write('seconds in flight,{},error,'.format(self.time))
        for i in range(6):
            f.write('{},'.format(err[0][i]))
        f.write('{}\n'.format(err[0][5]))
        f.close()
        
    def measure(self):
        'takes measurements and fills Y vector'
        #read IMU
        m = self.mpu.readMagnet()
        a = self.mpu.readAccel()
        mag = np.array([m['x'], m['y'], m['z']])
        accel = np.array([a['x'], a['y'], a['z']])

        self.dir = tiltCorrect(mag,accel)
        
        #read GPS
        data = [0,0]
        ser = serial.Serial('/dev/ttyS0', 4800, timeout = 0)
        for i in [0,1]:
            data[i] = ser.readline().decode('ascii').strip()
            for char in data[i]:
                self.gps.update(char)
        ser.close()
       
        #assume Northern + Western hemispheres
        lat = (self.gps.latitude[0] + (self.gps.latitude[1] / 60))
        long = - ( self.gps.longitude[0] + (self.gps.longitude[1] / 60))
        alt = self.gps.altitude

        t = self.gps.timestamp[0] * 3600 + self.gps.timestamp[1] * 60 + self.gps.timestamp[2]
        dt = t - self.time
        if dt != 0:
            self.dt = dt
        self.time = t

        #gps.speed is (knots, mph, km/h)
        v = self.gps.speed[2] * (1000/3600) #convert to m/s
        theta = -self.gps.course * pi/180
        vx = v * cos(theta)
        vy = v * sin(theta)
        try:
                vz = (alt - self.alt) / self.dt
        except:
                vz = 0
        self.Y = np.array([lat, long, alt, vx, vy, vz]).reshape([6,1])
        #NEED: Retrieve wind velocity for B matrix
                #Calculation of B matrix
                #Error estimation + logging

