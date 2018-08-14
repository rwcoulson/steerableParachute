from state import *
from time import time,sleep

t = time()
s = state()

for i in range(20):
	try:
		s.measure()
		s.Amatrix()
		R = np.eye(6)
		B = np.zeros([6,1])
		Y = np.array([[1,1,1,0,0,0]]).T
		s.kFilter(s.X, s.P, s.A, B, R, Y)
		s.logError()
		print(s.X)
		sleep(0.5)
	except KeyboardInterrupt:
		break
