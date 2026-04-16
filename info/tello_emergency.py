import sys
#sys.path.append(r'c:\users\giuseppe\appdata\local\packages\pythonsoftwarefoundation.python.3.10_qbz5n2kfra8p0\localcache\local-packages\python310\site-packages')

from djitellopy import Tello

tello = Tello()
tello.connect()
tello.land()
