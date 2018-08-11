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
            self.X = np.array([0,0,0,0,0,0])
        self.P = np.ones([6,6]) * 1000
        self.Y = [0,0,0,0,0,0]
        #initialize IMU and GPS objects
        self.mpu = MPU9250.MPU9250()
        self.gps = MicropyGPS()
        #implement: destination in external file

    def Amatrix(self, dt = 1, theta = 0):
        self.A = np.eye(6)
        for i in range(3):
            self.A[i][i+3] = dt

    def kFilter(Xprev, Pprev, A, Bu, R, Y):
        'performs a Kalman filter and updates the state/covariance vectors'
        I = np.eye(6)
        Xp = A @ Xprev + Bu
        Pp = A @ Pprev @ A.T
        for i in range(len(Pp)):
            for j in range(len(Pp[0])):
                if I[i][j] == 0:
                    Pp[i][j] == 0
        K = Pp / (Pp + R)
        X = Xp + K @ (Y - Xp)
        P = (I - K) @  Pp
        self.X = X
        self.P = P

    def measure():
        'takes measurements and fills Y vector'
        #read IMU
        m = self.mpu.readMagnet()
        a = self.mpu.readAccel()
        mag = np.array([m['x'], m['y'], m['z']])
        accel = np.array([a['x'], a['y'], a['z']])

        self.dir = tiltCorrect(mag,accel)
        #needs more processing
        
        #read GPS
        gpsData = [0,0]
        s = serial.Serial('/dev/ttyS0', 4800, timeout = 0)
        for i in [0,1]:
            data[i] = s.readline().decode('ascii').strip()
            for char in data[i]:
                gps.update(char)
        ser.close()
        
        #assume Northern + Western hemispheres
        lat = (gps.latitude[0] + (gps.latitude[1] / 60))
        long = - ( gps.longitude[0] + (gps.longitude[1] / 60))
        alt = gps.altitude

        t = gps.timestamp[0] * 3600 + gps.timestamp[1] * 60 + gps.timestamp[2]
        self.dt = t - self.time
        self.time = t
        
        v = gps.speed[2] * (1000/3600) #convert to m/s
        theta = -gps.course * pi/180
        vx = v * cos(theta)
        vy = v * sin(theta)
        vz = (alt - self.alt) / dt
        self.Y = [lat, long, alt, vx, vy, vz]
        #not finished