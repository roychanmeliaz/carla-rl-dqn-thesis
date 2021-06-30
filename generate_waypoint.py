#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


from draw_lane_line import point_pos
import glob
import os
import sys
import time
import settings


import atexit
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
_SLEEP_TIME_ = .1

waypoints = []

def get_dist(x1,y1,x2,y2):
    return math.hypot(x2 - x1, y2 - y1)

def exit_handler():
    global waypoints
    with open('waypoints.txt', 'w') as f:
        for waypoint in waypoints:
           f.write("bunderan_waypoints.append(({},{}))\n".format(waypoint.location.x, waypoint.location.y))
        print("Printed on waypoints.txt!")

def main():
    atexit.register(exit_handler)
    client = carla.Client(_HOST_, _PORT_)
    client.set_timeout(2.0)
    world = client.get_world()

    t = world.get_spectator().get_transform()
    global waypoints
    waypoints.append(t)
    print(t)

    while(True):
        t = world.get_spectator().get_transform()
        lastx = waypoints[-1].location.x
        lasty = waypoints[-1].location.y
        nowx = t.location.x
        nowy = t.location.y
        if get_dist(lastx,lasty,nowx,nowy) >= 1.0:
            waypoints.append(t)
            world.debug.draw_box(carla.BoundingBox(carla.Location(x=nowx,y=nowy,z=0),carla.Vector3D(0.1,0.1,0.2)),carla.Rotation(yaw=t.rotation.yaw), 0.05, carla.Color(255,0,0,0),20.0)
            print(t)
        
        # coordinate_str = "(x,y,z) = ({},{},{}) | (pitch,yaw,roll) = ({},{},{})".format(
        #     round(t.location.x,3), round(t.location.y,3), round(t.location.z,3),
        #     round(t.rotation.pitch,3), round(t.rotation.yaw,3), round(t.rotation.roll,3)
        #     )
        # print (coordinate_str)
        # time.sleep(_SLEEP_TIME_)

if __name__ == '__main__':
    main()


