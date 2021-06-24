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
_SLEEP_TIME_ = .5


def main():
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()
    
    # (x,y,z) = (2.238,-0.462,67.119) | (pitch,yaw,roll) = (-89.0,-178.717,0.0)
    camera = world.get_spectator()
    camera.set_transform(carla.Transform(carla.Location(x=0.0,y=0.0,z=50.0),carla.Rotation(pitch=-89.0,yaw=-180.0,roll=0.0)))

    while(True):
        camera.set_transform(carla.Transform(carla.Location(x=0.0,y=0.0,z=50.0),carla.Rotation(pitch=-89.0,yaw=-180.0,roll=0.0)))
        time.sleep(_SLEEP_TIME_)

if __name__ == '__main__':
    main()


