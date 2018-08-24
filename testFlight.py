from state import *

import os

s = state()

s.acquireGPS()
s.measure()
t = s.gps.timestamp
d = s.gps.date
flightID = '{}{}{}{}{}{}.csv'.format(d[0],d[1],d[2],t[0],t[1],t[2])
directory = 'flightData{}'.format(flightID)
os.mkdir(directory)
s.gps.start_logging('{}/gps_raw.csv'.format(directory))

while True:
    if s.burst = False:
        s.ascentLoop(directory)
    if s.burst = True:
        s.descentLoop(directory)
        s.logError('{}/error.csv'.format(directory, flightID))
