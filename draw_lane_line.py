#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time
import settings

import math
from math import sin, cos, radians, pi

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

def point_pos(x0, y0, d, theta):
    theta_rad = math.pi/2 - radians(theta)
    return x0 + d*cos(theta_rad), y0 + d*sin(theta_rad)

def main():
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()
    
    # (x,y,z) = (2.238,-0.462,67.119) | (pitch,yaw,roll) = (-89.0,-178.717,0.0)
    camera = world.get_spectator()
    camera.set_transform(carla.Transform(carla.Location(x=0.0,y=0.0,z=65.0),carla.Rotation(pitch=-89.0,yaw=-180.0,roll=0.0)))

    # get spawn points
    for i, _ in enumerate(range(359)):
        # xp,yp = point_pos(-.5,.5,21.5,i)
        xp,yp = point_pos(-.5,.5,30,i)
        print(xp,yp)
        # world.debug.draw_box(carla.BoundingBox(carla.Location(x = 0, y = 0, z=0),carla.Vector3D(0.1,0.1,20)),carla.Rotation(), 0.05, carla.Color(255,0,0,0),10.0)
        if i%2==0:
            world.debug.draw_box(carla.BoundingBox(carla.Location(x = xp, y = yp, z=0),carla.Vector3D(0.1,0.1,0.4)),carla.Rotation(), 0.2, carla.Color(255,0,0,0),30.0)
        # world.debug.draw_string(carla.Location(x = xp, y = yp, z=0), '.', draw_shadow=False,
        # 						color=carla.Color(r=255, g=255, b=255), life_time=300.0,
        # 						persistent_lines=True)

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


