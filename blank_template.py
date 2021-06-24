#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time
import settings

try:
    sys.path.append(glob.glob(settings.CARLA_PATH + f'/PythonAPI/carla/dist/carla-*{sys.version_info.major}.{sys.version_info.minor}-{"win-amd64" if os.name == "nt" else "linux-x86_64"}.egg')[0])
except IndexError:
    pass
# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

_HOST_ = '127.0.0.1'
_PORT_ = 2000
_SLEEP_TIME_ = .1


def main():
	client = carla.Client(_HOST_, _PORT_)
	client.set_timeout(2.0)
	world = client.get_world()

	while(True):
		t = world.get_spectator().get_transform()
		coordinate_str = "(x,y,z) = ({},{},{}) | (pitch,yaw,roll) = ({},{},{})".format(
			round(t.location.x,3), round(t.location.y,3), round(t.location.z,3),
			round(t.rotation.pitch,3), round(t.rotation.yaw,3), round(t.rotation.roll,3)
			)
		print (coordinate_str)
		time.sleep(_SLEEP_TIME_)

if __name__ == '__main__':
	main()


