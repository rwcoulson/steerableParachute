from state import *

s = state()

s.acquireGPS()


while True:
    if s.burst = False:
        s.ascentLoop()
    if s.burst = True:
        s.descentLoop()
        s.logError()
