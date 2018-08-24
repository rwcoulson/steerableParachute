from state import *

import os

s = state()


s.acquireGPS()
s.measure()
s.Y = s.X
t = s.gps.timestamp
s.time = self.gps.timestamp[0] * 3600 + self.gps.timestamp[1] * 60 + self.gps.timestamp[2]
d = s.gps.date
flightID = '{}{}{}{}{}{}.csv'.format(d[0],d[1],d[2],t[0],t[1],t[2])
directory = 'flightData{}'.format(flightID)
initFiles(directory)
os.mkdir(directory)

while True:
    if s.burst = False:
        s.ascentLoop(directory)
    if s.burst = True:
        s.descentLoop(directory)
        s.logError('{}/error.csv'.format(directory, flightID))
